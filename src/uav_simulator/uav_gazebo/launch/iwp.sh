#! /bin/bash

ros2 launch uav_gazebo spawn_iwp.launch &
sleep 5
echo "gazebo starting success!"

ros2 launch uav_gazebo apm.launch &
sleep 1

# ros2 run uav_control iwp_claw_node &
# sleep 1

wait 
exit 0
