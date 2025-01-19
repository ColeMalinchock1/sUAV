#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <serial/serial.h>

// Constants
namespace D1_Const {
    constexpr uint16_t IMAGE_WIDTH = 160;
    constexpr uint16_t IMAGE_HEIGHT = 60;
    constexpr uint16_t DISTANCE_MAX_VALUE_3D = 10000;  // 10m
    constexpr uint16_t ADC_OVERFLOW_3D = 16000;
    constexpr uint16_t SATURATION_3D = 16001;
}

namespace ROS_Const {
    constexpr float PIXEL_REAL_SIZE = 0.1f;
    constexpr float OFFSET_CENTER_POINT_X = 0.0f;
    constexpr float OFFSET_CENTER_POINT_Y = 0.0f;
    constexpr float MM2M = 0.001f;
    
    struct Color {
        uint8_t R, G, B, A;
    };
    
    const Color ADC_OVERFLOW_COLOR = {255, 0, 0, 255};
    const Color SATURATION_COLOR = {0, 255, 0, 255};
    const Color NONE_COLOR = {0, 0, 0, 0};
    
    enum ColorMode {
        MODE_HUE = 0,
        MODE_RGB = 1,
        MODE_GRAY = 2
    };
}

class CYG_Distortion {
public:
    void initLensTransform(float pixel_size, uint16_t width, uint16_t height, 
                          float offset_x, float offset_y) {
        pixel_size_ = pixel_size;
        width_ = width;
        height_ = height;
        offset_x_ = offset_x;
        offset_y_ = offset_y;
    }
    
    void transformPixel(uint32_t index, uint16_t distance, 
                       float& x, float& y, float& z) {
        uint16_t pixel_x = index % width_;
        uint16_t pixel_y = index / width_;
        
        x = (pixel_x - width_ / 2.0f) * pixel_size_ + offset_x_;
        y = (pixel_y - height_ / 2.0f) * pixel_size_ + offset_y_;
        z = distance;
    }

private:
    float pixel_size_;
    uint16_t width_, height_;
    float offset_x_, offset_y_;
};

class CygLidarNode : public rclcpp::Node {
public:
    CygLidarNode() : Node("cyglidar_node") {
        // Declare parameters
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 3000000);
        this->declare_parameter("frame_id", "cyglidar_link");
        this->declare_parameter("color_mode", 0);  // HUE mode by default

        // Get parameters
        port_ = this->get_parameter("port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        color_mode_ = this->get_parameter("color_mode").as_int();

        // Create publishers
        publisher_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "cyglidar/points", 10);
        publisher_image_ = this->create_publisher<sensor_msgs::msg::Image>(
            "cyglidar/depth_image", 10);

        // Initialize PCL cloud
        pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl_cloud_->header.frame_id = frame_id_;
        pcl_cloud_->is_dense = false;
        pcl_cloud_->width = D1_Const::IMAGE_WIDTH;
        pcl_cloud_->height = D1_Const::IMAGE_HEIGHT;
        pcl_cloud_->points.resize(D1_Const::IMAGE_WIDTH * D1_Const::IMAGE_HEIGHT);

        // Initialize distortion correction
        distortion_.initLensTransform(ROS_Const::PIXEL_REAL_SIZE, 
                                    D1_Const::IMAGE_WIDTH, 
                                    D1_Const::IMAGE_HEIGHT,
                                    ROS_Const::OFFSET_CENTER_POINT_X, 
                                    ROS_Const::OFFSET_CENTER_POINT_Y);

        // Initialize color mapping
        initColorMap(color_mode_);

        // Initialize serial connection
        try {
            serial_.setPort(port_);
            serial_.setBaudrate(baud_rate_);
            serial_.setBytesize(serial::eightbits);
            serial_.setParity(serial::parity_none);
            serial_.setStopbits(serial::stopbits_one);
            serial_.setFlowcontrol(serial::flowcontrol_none);
            serial_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            return;
        }

        // Start scanning
        startScanning();

        // Create timer for reading data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&CygLidarNode::readData, this));
    }

private:
    void startScanning() {
        uint8_t start_cmd[] = {0x5A, 0x77, 0xFF, 0x02, 0x00, 0x01, 0x00, 0x03};
        serial_.write(start_cmd, sizeof(start_cmd));
    }

    void readData() {
        if (!serial_.available()) {
            return;
        }

        // Read data frame
        std::vector<uint16_t> distance_buffer(D1_Const::IMAGE_WIDTH * D1_Const::IMAGE_HEIGHT);
        
        // Read and process the data frame here
        // Note: Actual frame reading code would go here - this is simplified
        
        publishDepthImage(distance_buffer);
        publishPointCloud(distance_buffer);
    }

    void publishDepthImage(const std::vector<uint16_t>& distance_buffer) {
        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header.frame_id = frame_id_;
        msg->header.stamp = this->now();
        msg->height = D1_Const::IMAGE_HEIGHT;
        msg->width = D1_Const::IMAGE_WIDTH;
        msg->encoding = sensor_msgs::image_encodings::RGBA8;
        msg->step = msg->width * 4;
        msg->is_bigendian = false;
        msg->data.resize(msg->height * msg->step);

        for (size_t i = 0; i < distance_buffer.size(); ++i) {
            size_t step = (i % D1_Const::IMAGE_WIDTH) * 4 + 
                         (i / D1_Const::IMAGE_WIDTH) * msg->step;
            
            uint16_t distance = distance_buffer[i];
            auto color = getColorForDistance(distance);
            
            msg->data[step] = color.R;
            msg->data[step + 1] = color.G;
            msg->data[step + 2] = color.B;
            msg->data[step + 3] = color.A;
        }

        publisher_image_->publish(*msg);
    }

    void publishPointCloud(const std::vector<uint16_t>& distance_buffer) {
        pcl_conversions::toPCL(this->now(), pcl_cloud_->header.stamp);

        for (size_t i = 0; i < distance_buffer.size(); ++i) {
            uint16_t distance = distance_buffer[i];
            
            if (distance < D1_Const::DISTANCE_MAX_VALUE_3D) {
                float x, y, z;
                distortion_.transformPixel(i, distance, x, y, z);

                pcl_cloud_->points[i].x = z * ROS_Const::MM2M;
                pcl_cloud_->points[i].y = -x * ROS_Const::MM2M;
                pcl_cloud_->points[i].z = -y * ROS_Const::MM2M;

                auto color = getColorForDistance(distance);
                uint32_t rgb = ((uint32_t)color.R << 16 | 
                               (uint32_t)color.G << 8 | 
                               (uint32_t)color.B);
                pcl_cloud_->points[i].rgb = *reinterpret_cast<float*>(&rgb);
                pcl_cloud_->points[i].a = color.A;
            } else {
                pcl_cloud_->points[i].x = 0;
                pcl_cloud_->points[i].y = 0;
                pcl_cloud_->points[i].z = 0;
                pcl_cloud_->points[i].a = 0;
            }
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*pcl_cloud_, cloud_msg);
        publisher_cloud_->publish(cloud_msg);
    }

    ROS_Const::Color getColorForDistance(uint16_t distance) {
        if (distance == D1_Const::ADC_OVERFLOW_3D) {
            return ROS_Const::ADC_OVERFLOW_COLOR;
        } else if (distance == D1_Const::SATURATION_3D) {
            return ROS_Const::SATURATION_COLOR;
        } else if (distance >= D1_Const::DISTANCE_MAX_VALUE_3D) {
            return ROS_Const::NONE_COLOR;
        }

        size_t color_index = (distance / color_gap_) % color_map_.size();
        return color_map_[color_index];
    }

    void initColorMap(int color_mode) {
        // Implementation of color map initialization based on color_mode
        // This would follow the same logic as the original Topic3D::initColorMap()
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial serial_;
    std::string port_;
    int baud_rate_;
    std::string frame_id_;
    int color_mode_;
    
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> pcl_cloud_;
    CYG_Distortion distortion_;
    std::vector<ROS_Const::Color> color_map_;
    float color_gap_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CygLidarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}