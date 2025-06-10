#!/bin/bash

source /environment.sh

dt-launchfile-init
rosrun my_package wheel_control_node.py
dt-launchfile-join