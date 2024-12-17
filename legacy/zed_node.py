import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import threading
import mediapipe as mp
import curses

# Setup the printing screen for debugging
stdscr = curses.initscr()

# Sets the frame to None
frame = None

# For debugging purposes
debugger = None

# Sets the cv bridge and hand identifier
br = CvBridge()
mp_hands = mp.solutions.hands.Hands()

# Callback for the images from the ZED
def image_callback(msg):
    global frame
    frame = br.imgmsg_to_cv2(msg)

# Crops the main image depending on the targets detection in left and right cameras
def crop_main(image, width, lx, rx):
    global debugger
    if lx == 0 and rx == 0:
        pass
    elif lx == 0:
        image = crop_right(image, width)
    elif rx == 0:
        image = crop_left(image, width)
    else:
        center = int(width / 2)
        debugger = [lx, center + rx]
        image = np.delete(image, slice(lx - 50, center +  rx - 50), 1)
    return image

# Crops the image to just get left camera image
def crop_left(image, width):
    center = int(width / 2)

    image = np.delete(image, slice(center, width), 1)
    return image

# Crops the image to just get right camera image
def crop_right(image, width):
    center = int(width / 2)

    image = np.delete(image, slice(0, center), 1)
    return image

# Uses mediapipe to detect the hand
def detect_hand(img):

    # Converts the color of the hand to rgb
    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)

    results = mp_hands.process(img)

    cx, cy = 0, 0

    # Ff hand is detected, get the x and y coordinates of it
    if results.multi_hand_landmarks:
        hand_detected = True

        lm_list = []
        myHand = results.multi_hand_landmarks[0]

        for id, lm in enumerate(myHand.landmark):
            h, w, c = img.shape
            cx, cy = int(lm.x * w), int(lm.y * h)
            
    else:
        hand_detected = False
    
    return cx, cy

# Process the images of the ZED
def process_image():
    global frame

    img = frame

    height, width, _ = img.shape

    # Crop the left and right of the original image to jsut show left and right camera individually
    cropped_left_image = crop_left(img, width)
    cropped_right_image = crop_right(img, width)

    # Flip the left and the right images
    flipped_left_image = cv.flip(cropped_left_image, 0)
    flipped_right_image = cv.flip(cropped_right_image, 0)

    # Detect the hands in the left and the right images and get their locations
    lx, ly = detect_hand(flipped_left_image)
    rx, ry = detect_hand(flipped_right_image)

    # Depending on the location of the target, crop the frame accordingly
    cropped_main_image = crop_main(img, width, lx, rx)
    flipped_main_image = cv.flip(cropped_main_image, 0)

    # Show the images
    #cv.imshow("Original", cv.flip(frame, 0))
    cv.imshow("Main frame", flipped_main_image)
    #cv.imshow("Left cropped", flipped_left_image)
    #cv.imshow("Right cropped", flipped_right_image)
    cv.waitKey(1)

    return lx, ly, rx, ry

# Main
def main(args = None):
    global debugger

    # Initialize the coordinates of the target
    lx = 0
    ly = 0
    rx = 0
    ry = 0
    
    # Initialize the ros2 node
    rclpy.init(args = args)
    node = Node("zed_node")

    # Create the subscription to the zed camera
    zed_img_subscription = node.create_subscription(Image, '/zed/zed_node/stereo/image_rect_color', image_callback, 5)

    # Start threading the ros2 node
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()

    # Have the node run at 20Hz
    FREQ = 20
    rate = node.create_rate(FREQ, node.get_clock())

    # Loop while the ros2 node is ok
    while rclpy.ok():

        # Check that there is a frame
        if frame is not None:
            frame_received = True
            lx, ly, rx, ry = process_image()
        else:
            frame_received = False
            
        # Output to terminal for debugging
        stdscr.refresh()
        stdscr.addstr(1, 5, 'Debugger: %s         ' % str(debugger))
        stdscr.addstr(2, 5, 'Frame Receieved: %s         ' % frame_received)
        stdscr.addstr(4, 5, 'Left Frame X: %s            ' % str(lx))
        stdscr.addstr(5, 5, 'Left Frame Y: %s            ' % str(ly))
        stdscr.addstr(6, 5, 'Right Frame X: %s            ' % str(rx))
        stdscr.addstr(7, 5, 'Right Frame Y: %s            ' % str(ry))

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()