#!/usr/bin/bash
rostopic pub /goToGoal geometry_msgs/Pose2D -- '$1' '$2' '$3'
