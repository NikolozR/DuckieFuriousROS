#!/usr/bin/env python3


import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped


# Twist command parameters
VELOCITY = 0.3  # linear m/s, forward (+)
OMEGA    = 4.0  # angular rad/s, CCW (+)


class TwistControlNode(DTROS):


   def __init__(self, node_name):
       super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
       vehicle_name = os.environ['VEHICLE_NAME']
       twist_topic  = f"/{vehicle_name}/car_cmd_switch_node/cmd"
       self._v     = VELOCITY
       self._omega = OMEGA
       self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)


   def run(self):
       rate = rospy.Rate(10)
       message = Twist2DStamped(v=self._v, omega=self._omega)
       while not rospy.is_shutdown():
           self._publisher.publish(message)
           rate.sleep()


   def on_shutdown(self):
       stop = Twist2DStamped(v=0.0, omega=0.0)
       self._publisher.publish(stop)


if __name__ == '__main__':
   node = TwistControlNode(node_name='twist_control_node')
   node.run()
   rospy.spin()