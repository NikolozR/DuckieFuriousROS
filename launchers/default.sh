#!/bin/bash
 
source /environment.sh
 
# initialize launch file
dt-launchfile-init
 
# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
 
 
# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)
 
# launching app
dt-exec rosrun Main camera_node.py
dt-exec rosrun Main wheel_control_node.py
# dt-exec rosrun Main hsv_calibrator.py
dt-exec rosrun Main coordinatooor.py
 
# dt-exec rosrun Main sign_detectooor.py
 
 
 
# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE
 
# wait for app to end
dt-launchfile-join