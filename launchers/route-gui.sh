#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# On your laptop, with intersections.sh already running on the robot.
# THIS ONE RUNS ON THE LAPTOP (-R: talk to the robot's ROS master, -X: show windows):
#
#     dts devel build -f
#     dts devel run -R DUCKIEBOT_NAME -L route-gui -X
#
# No robot: run the planner here too. Anything after ++ goes to roslaunch.
#
#     dts devel run -L route-gui -X ++ test:=true
#
dt-exec roslaunch intersection_control route_gui.launch "$@"


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
