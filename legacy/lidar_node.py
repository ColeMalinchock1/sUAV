import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import threading

SHOW_IMAGES = False
SHOW_3D = False
SHOW_2D = False
SHOW_SCAN = True
points_3d_array = point_2d_array = frame = None
br = CvBridge()
obstacle_location = "None detected"

def scan_3d_callback(msg):
    global points_3d_array
    points = []
    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([point[0], point[1], point[2]])
    
    points_array = np.array(points)

def scan_2d_callback(msg):
    pass

def scan_callback(msg):
    print(len(msg.ranges))

def scan_image_callback(msg):
    global frame
    frame = br.imgmsg_to_cv2(msg)                                                                                                                                    

def process_image(frame):

	img = frame
	
	height , width , _ = img.shape # 720 , 2560

	main_image = img
	color_frame_main = main_image

	bw_frame_main = cv.cvtColor(main_image , cv.COLOR_BGR2GRAY)
	bw_frame_main = main_image

	height , width, _ = main_image.shape  # Cropped size

	# Remove noise
	kernel_size = 7
	bw_frame_main = cv.GaussianBlur(bw_frame_main , (kernel_size , kernel_size) , 0)

	# Thresholding. If seeing some noise, increase the lower threshold
	lower_threshold = 160
	upper_threshold = 255
	_, bw_frame_main = cv.threshold(bw_frame_main , lower_threshold , upper_threshold , cv.THRESH_BINARY)


	# Option of showing all the images, can be toggled at top
	if SHOW_IMAGES:
		cv.imshow('main color' , frame)
		cv.imshow('main black and white' , bw_frame_main)
		cv.waitKey(1)

def find_obstruction(array):
    centroid = np.mean(array, axis = 0)

    left = array[:, 0] < centroid[0]
    right = array[:, 0] > centroid[0]
    top = array[:, 1] > centroid[1]
    bottom = array[:, 1] < centroid[1]

    left_count = np.sum(left)
    right_count = np.sum(right)
    top_count = np.sum(top)
    bottom_count = np.sum(bottom)

    directions = {
        'left': left_count,
        'right': right_count,
        'top': top_count,
        'bottom': bottom_count
    }

    obstruction_direction = max(directions, key = directions.get)
    return obstruction_direction


def main(args = None):
    global points_array, frame

    rclpy.init(args = args)
    node = Node("lidar_node")

    scan_subscription = node.create_subscription(LaserScan, '/scan', scan_callback, 5)

    scan_2d_subscription = node.create_subscription(PointCloud2, '/scan_2D', scan_2d_callback, 5)

    scan_3d_subscription = node.create_subscription(PointCloud2, '/scan_3D', scan_3d_callback, 5)

    img_subscription = node.create_subscription(Image, '/scan_image', scan_image_callback, 5)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    FREQ = 20
    rate = node.create_rate(FREQ, node.get_clock())

    while rclpy.ok():

        if SHOW_3D:
            if points_3d_array is not None:
                pass
                #find_obstruction(points_array)
            else:
                print("No points received")
        if SHOW_IMAGES:
            if frame is not None:
                process_image(frame)
            else:
                print("No frame received")
        if SHOW_2D:
             pass
        if SHOW_SCAN:
             pass
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()