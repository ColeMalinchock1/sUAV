import numpy as np
import cv2

class DepthProcessing():

    def __init__(self, real_time=True, directory=None):
        self.real_time = real_time
        self.directory = directory


    def process_rgb_image(self, image):
        if not self.real_time:
            image = cv2.imread(f"{self.directory/{image}}")
        self.rgb_image = image


    def process_depth_np(self, array):
        if not self.real_time:
            array = np.load(f"{self.directory}/{array}", "r")
        print(array)
        self.depth_image = array

    def visualize_depth(self, image):
        cv2.imshow("Depth Image", image)

    def visualize_rgb(self, image):
        cv2.imshow("RGB Image", image)


def main():

    depth_processing = DepthProcessing(real_time=False, directory="results")



if __name__ == "__main__":
    main()