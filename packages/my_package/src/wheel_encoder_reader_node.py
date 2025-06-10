#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped

class WheelEncoderReaderNode(DTROS):

   def __init__(self, node_name):
       super(WheelEncoderReaderNode, self).__init__(
           node_name=node_name,
           node_type=NodeType.PERCEPTION
       )
       self._vehicle_name = os.environ['VEHICLE_NAME']
       self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
       self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
       self._ticks_left = None
       self._ticks_right = None
       self.sub_left = rospy.Subscriber(
           self._left_encoder_topic,
           WheelEncoderStamped,
           self.callback_left
       )
       self.sub_right = rospy.Subscriber(
           self._right_encoder_topic,
           WheelEncoderStamped,
           self.callback_right
       )

   def callback_left(self, data):
       rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
       rospy.loginfo_once(f"Left encoder type: {data.type}")
       self._ticks_left = data.data

   def callback_right(self, data):
       rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
       rospy.loginfo_once(f"Right encoder type: {data.type}")
       self._ticks_right = data.data

   def run(self):
       rate = rospy.Rate(20)
       while not rospy.is_shutdown():
           if self._ticks_left is not None and self._ticks_right is not None:
               msg = (
                   f"Wheel encoder ticks [LEFT, RIGHT]: "
                   f"{self._ticks_left}, {self._ticks_right}"
               )
               rospy.loginfo(msg)
           rate.sleep()

if __name__ == '__main__':
   node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
   node.run()
   rospy.spin()