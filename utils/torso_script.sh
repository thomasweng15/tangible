#!/bin/bash

rostopic pub /torso_controller/position_joint_action/goal pr2_controllers_msgs/SingleJointPositionActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  position: $1
  min_duration:
    secs: 0
    nsecs: 0
  max_velocity: 0.0" --once