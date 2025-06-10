#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge
import numpy as np
from typing import Tuple



class CameraReaderNode(DTROS):

    def detect_lane_markings(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        image = cv2.bilateralFilter(image, 12, 125, 155)
        
        height, width, _ = image.shape
        left_half = image[:, :width//2]
        right_half = image[:, width//2:]     

        right_half_gray = cv2.cvtColor(right_half, cv2.COLOR_BGR2GRAY)  

        yellow_lower_color = np.array([9, 100, 0])
        yellow_upper_color = np.array([80, 255, 255])
        # white_lower_color = np.array([0, 177, 0])
        # white_upper_color = np.array([255, 255, 120])

        hsv_left = cv2.cvtColor(left_half, cv2.COLOR_BGR2HSV)
        # hls_right = cv2.cvtColor(right_half, cv2.COLOR_BGR2HLS)
        # h_channel, l_channel, s_channel = cv2.split(hls_right)


        mask_yellow_left = cv2.inRange(hsv_left, yellow_lower_color, yellow_upper_color)
        # mask_white_right = cv2.inRange(hls_right, white_lower_color, white_upper_color)

        result_left = cv2.bitwise_and(left_half, left_half, mask=mask_yellow_left)
        # result_right = cv2.bitwise_and(right_half, right_half, mask=mask_white_right)
        # result_right = cv2.threshold(gray, 62, 255, cv2.THRESH_BINARY)
        # Apply binary threshold to the grayscale image
        _, binary = cv2.threshold(right_half_gray, 177, 255, cv2.THRESH_BINARY)
        binary_rgb = cv2.merge([binary, binary, binary])
        result_right = cv2.bitwise_and(right_half, binary_rgb)

        result_left[:height//3] = 0
        result_right[:height//3] = 0

        return result_left, result_right

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "camera-reader"
        self._window2 = "second-camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        # construct publisher
        self.pub = rospy.Publisher("eyes", CompressedImage, queue_size=10)
        self.pub2 = rospy.Publisher("eyes2", CompressedImage, queue_size=10)


    def callback(self, msg):
        # convert JPEG bytes to CV image
        self.pub2.publish(msg)
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        left, right = self.detect_lane_markings(image)
        concatenated_img = np.hstack((left, right))
        concatenated_img_msg = self._bridge.cv2_to_compressed_imgmsg(concatenated_img)
        # publish the image
        self.pub.publish(concatenated_img_msg)
        # display frame
        cv2.imshow(self._window, concatenated_img)
        cv2.imshow(self._window2, image)
        cv2.waitKey(1)

if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()