#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge
import numpy as np

# throttle and direction for each wheel
THROTTLE_LEFT = 0.5        # 50% throttle
DIRECTION_LEFT = 1         # forward
THROTTLE_RIGHT = 0.75       # 30% throttle
DIRECTION_RIGHT = 1       # backward

CONST = 0.4  # Increased base speed to overcome friction
GAIN = 0.8   # Doubled gain for more responsive turning
LEFT = 0.4   # Doubled left sensitivity
RIGHT = 0.4  # Doubled right sensitivity

class WheelControlNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._bridge = CvBridge()
        # form the message
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        self.sub = rospy.Subscriber("eyes", CompressedImage, self.callback)
        self.sub2 = rospy.Subscriber("eyes2", CompressedImage, self.callback2)
        self.stopped = False

    def callback2(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        self.stopped = not self.stopped
        # rospy.loginfo(f"{image.shape}")

        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        red_mask = cv2.bitwise_or(mask1, mask2)

        output_frame = cv2.bitwise_and(image, image, mask=red_mask)

        h, _, _ = image.shape
        start_row = 14 * h // 15
        bottom_third = np.zeros_like(output_frame)
        bottom_third[start_row:, :] = output_frame[start_row:, :]

        black_pixel = [0, 0, 0]
        non_black_indices = np.nonzero(np.any(output_frame[start_row:] != black_pixel, axis=-1))
        red_appears = len(non_black_indices[0]) > 0        
    
        if red_appears and self._red:
            # rospy.loginfo("Non-black pixels detected in the bottom third of the image")
            self._red = False
            self._vel_left = 0
            self._vel_right = 0
            message = WheelsCmdStamped(vel_left=0, vel_right=0)
            self._publisher.publish(message)
            self.stopped = True
            red_appears = False
            rospy.sleep(1)
            self._vel_left = 0.4
            self._vel_right = 0.1
            message = WheelsCmdStamped(vel_left=0.4, vel_right=0.1)
            self._publisher.publish(message)
            rospy.sleep(3)
        elif red_appears and not self._red:
            self._vel_left = 0.4
            self._vel_right = 0.1
            message = WheelsCmdStamped(vel_left=0.4, vel_right=0.1)
        elif not red_appears:
            self._red = True
            self.stopped = False

    def callback(self, msg):
        if self.stopped:
            return

        concatenated_images = self._bridge.compressed_imgmsg_to_cv2(msg)
        height, width, _ = concatenated_images.shape
        left_image = concatenated_images[:, :width//2]
        right_image = concatenated_images[:, width//2:]

        # Convert images to grayscale (if they are not already)
        left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
        
        # Count the non-zero elements in each half
        max_count = height * width
        left_non_zero_count = cv2.countNonZero(left_gray) / max_count
        right_non_zero_count = cv2.countNonZero(right_gray) / max_count

        left_motor = CONST + GAIN * left_non_zero_count * LEFT
        right_motor = CONST + GAIN * right_non_zero_count * RIGHT

        if(left_non_zero_count > right_non_zero_count):
            right_motor -= 0.3  # Increased from 0.2 for stronger right turn
            right_motor = 0.15 if right_motor < 0.15 else right_motor  # Lower minimum for sharper turns

            left_motor += 0.4  # Increased from 0.25 for stronger right turn
            left_motor = 0.4 if left_motor > 0.4 else left_motor  # Higher maximum for stronger turns

            message = WheelsCmdStamped(vel_left=left_motor, vel_right=right_motor)
            self._publisher.publish(message)

        elif(left_non_zero_count < right_non_zero_count):
            left_motor -= 0.3  # Increased from 0.2 for stronger left turn
            left_motor = 0.15 if left_motor < 0.15 else left_motor  # Lower minimum for sharper turns

            right_motor += 0.4  # Increased from 0.25 for stronger left turn
            right_motor = 0.4 if right_motor > 0.4 else right_motor  # Higher maximum for stronger turns

            message = WheelsCmdStamped(vel_left=left_motor, vel_right=right_motor)
            self._publisher.publish(message)   
        
        else:
            message = WheelsCmdStamped(vel_left=left_motor, vel_right=right_motor)
            self._publisher.publish(message) 

        self._vel_left = left_motor
        self._vel_right = right_motor

        rospy.loginfo(f"I heard {left_motor}, {right_motor}, {self.stopped}")


    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(0.1)
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        while not rospy.is_shutdown():
            self._publisher.publish(message)
            rate.sleep()

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':

    node = WheelControlNode(node_name='wheel_control_node')
    node.run()
    rospy.spin()