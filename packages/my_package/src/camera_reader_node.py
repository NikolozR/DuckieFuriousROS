#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

   def __init__(self, node_name):
       super(CameraReaderNode, self).__init__(
           node_name=node_name,
           node_type=NodeType.VISUALIZATION
       )
       self._vehicle_name = os.environ['VEHICLE_NAME']
       self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
       self._bridge = CvBridge()
       self._window = "camera-reader"
       cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
       self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

   def callback(self, msg):
       image = self._bridge.compressed_imgmsg_to_cv2(msg)
       cv2.imshow(self._window, image)
       cv2.waitKey(1)

if __name__ == '__main__':
   node = CameraReaderNode(node_name='camera_reader_node')
   rospy.spin()