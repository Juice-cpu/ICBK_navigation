#!/bin/bash

HOME_X=0.0
HOME_Y=0.0
HOME_Z=0.0


ANGULAR_SPEED=1  # rad/s


ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: $HOME_X, y: $HOME_Y, z: $HOME_Z}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" --once

while true; do
   
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ANGULAR_SPEED}}" --once
    sleep 1  
done
