#!/bin/bash

source /environment.sh

dt-launchfile-init
rosrun my_package traffic_sign_node.py
dt-launchfile-join 