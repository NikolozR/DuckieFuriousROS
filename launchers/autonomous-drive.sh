source /environment.sh

dt-launchfile-init

rosrun my_package camera_node.py &

rosrun my_package wheel_control_node_main.py &

wait

dt-launchfile-join 