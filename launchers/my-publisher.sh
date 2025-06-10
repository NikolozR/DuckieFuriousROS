#!/bin/bash
source /environment.sh
dt-launchfile-init
rosrun my_package my_publisher_node.py
dt-launchfile-join