#!/usr/bin/env python3


import os
import rospy
from std_msgs.msg import String
from duckietown.dtros import DTROS, NodeType


class MyPublisherNode(DTROS):
   def __init__(self, node_name):
       super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
       self._vehicle_name = os.environ['VEHICLE_NAME']
       self._publisher = rospy.Publisher('chatter', String, queue_size=10)


   def run(self):
       rate = rospy.Rate(1)  # 1 Hz
       message = f"Hello from {self._vehicle_name}!"
       while not rospy.is_shutdown():
           rospy.loginfo(f"Publishing message: '{message}'")
           self._publisher.publish(message)
           rate.sleep()


if __name__ == '__main__':
   node = MyPublisherNode(node_name='my_publisher_node')
   node.run()
   rospy.spin()