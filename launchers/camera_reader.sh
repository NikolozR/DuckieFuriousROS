#!/bin/bash

source /environment.sh

dt-launchfile-init
rosrun my_package camera_reader_node.py
dt-launchfile-join