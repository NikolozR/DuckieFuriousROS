#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np
from typing import Tuple
import cv2

# Twist command for controlling the linear and angular velocity of the frame
VELOCITY = 0.1  # linear vel    , in m/s    , forward (+)
OMEGA = 4.0     # angular vel   , rad/s     , counter clock wise (+)



class TwistControlNode(DTROS):
    def get_steer_matrix_left_lane_markings(self, shape: Tuple[int, int]) -> np.ndarray:
        """
        Args:
            shape:              The shape of the steer matrix.
    
        Return:
            steer_matrix_left:  The steering (angular rate) matrix for Braitenberg-like control
                                using the masked left lane markings (numpy.ndarray)
        """
    
        steer_matrix_left = np.zeros(shape)
        steer_matrix_left[:,shape[1]//3:] = -1
        # ---
        return steer_matrix_left
 
 
    def get_steer_matrix_right_lane_markings(self, shape: Tuple[int, int]) -> np.ndarray:
        """
        Args:
            shape:               The shape of the steer matrix.
    
        Return:
            steer_matrix_right:  The steering (angular rate) matrix for Braitenberg-like control
                                using the masked right lane markings (numpy.ndarray)
        """
    
        steer_matrix_right = np.zeros(shape)
        steer_matrix_right[:,:shape[1] *2//3] = 1
        return steer_matrix_right
    
    
    def detect_lane_markings(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
            right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
        """
        h, w, _ = image.shape
    
        consta = 100
        white_lower_color = np.array([0,0,255-consta])
        white_upper_color = np.array([255,consta,255])
        yellow_lower_color = np.array([10,50,80])
        yellow_upper_color = np.array([30,255,255])
    
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        img_gray_scale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    
        gaussian_blur = cv2.GaussianBlur(img_gray_scale, (0,0), np.pi)
        sobel_x_crd = cv2.Sobel(gaussian_blur, cv2.CV_64F,1,0)
        sobel_y_crd = cv2.Sobel(gaussian_blur, cv2.CV_64F,0,1)
        G = np.sqrt(sobel_x_crd*sobel_x_crd + sobel_y_crd*sobel_y_crd)
    
        threshold_w = 27
        threshold_y = 47
    
        mask_bool_white = (G > threshold_w)
        mask_bool_yellow = (G > threshold_y)
    
        real_mask_white = cv2.inRange(img_hsv, white_lower_color, white_upper_color)
        real_mask_yellow = cv2.inRange(img_hsv, yellow_lower_color, yellow_upper_color)
    
        left_mask = np.ones(sobel_x_crd.shape)
        left_mask[:,int(np.floor(w/2)):w+1] = 0
        right_mask = np.ones(sobel_x_crd.shape)
        right_mask[:,0:int(np.floor(w/2))] = 0
        left_mask[:int(sobel_x_crd.shape[0]/2)] = 0
        right_mask[:int(sobel_x_crd.shape[0]/2)] = 0
        sobel_x_pos_mask = (sobel_x_crd > 0)
        sobel_x_neg_mask = (sobel_x_crd < 0)
        sobel_y_pos_mask = (sobel_y_crd > 0)
        sobel_y_neg_mask = (sobel_y_crd < 0)
    
        left_edge_mask = left_mask * mask_bool_yellow * sobel_x_neg_mask * sobel_y_neg_mask * real_mask_yellow
        right_edge_mask = right_mask * mask_bool_white * sobel_x_pos_mask * sobel_y_neg_mask * real_mask_white
    
        return  left_edge_mask, right_edge_mask

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        self._bridge = CvBridge()
        # form the message
        self._v = VELOCITY
        self._omega = OMEGA
        # construct publisher
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self.sub = rospy.Subscriber("eyes", CompressedImage, self.callback)

    def callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        left, right = self.detect_lane_markings(image)
        left_mask, right_mask = self.get_steer_matrix_left_lane_markings(left.shape), self.get_steer_matrix_right_lane_markings(right.shape)
        steering = np.sum(left_mask * left) + np.sum(right_mask * right)

        self._omega = steering

        rospy.loginfo("I heard '%s'", self._omega)

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        message = Twist2DStamped(v=self._v, omega=self._omega)
        while not rospy.is_shutdown():
            self._publisher.publish(message)
            rate.sleep()

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = TwistControlNode(node_name='twist_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()