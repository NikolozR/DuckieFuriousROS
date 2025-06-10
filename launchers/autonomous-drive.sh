#!/bin/bash

source /environment.sh

dt-launchfile-init

# Step 1: Launch camera reader to process raw camera feed and provide 'eyes' topic
rosrun my_package camera_reader_node.py &

# Step 2: Launch wheel encoder for feedback control
rosrun my_package wheel_encoder_reader_node.py &

# Step 3: Launch lane following (base driving behavior)
rosrun my_package twist_control_node.py &

# Step 4: Launch traffic sign detection (overrides velocity when signs detected)
rosrun my_package traffic_sign_node.py &

# Note: Both twist_control and traffic_sign publish to the same cmd topic
# Traffic signs will override with stop/slow behavior when detected
# When no signs, twist_control provides lane following

# Wait for all background processes
wait

dt-launchfile-join 