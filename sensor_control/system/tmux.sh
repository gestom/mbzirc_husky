tmux rename-window -t 0 "Devices"
tmux send-keys 'rostopic echo /status | grep battery_voltage'
tmux split-window -v
sleep 0.2
tmux send-keys 'roslaunch mbzirc_husky robot.launch'
tmux split-window -h
sleep 0.2
tmux send-keys 'roslaunch mbzirc_husky velodyne.launch'
tmux split-window 
sleep 0.2
tmux send-keys 'roslaunch realsense2_camera rs_camera.launch'
tmux select-layout tiled
sleep 0.2
tmux new-window -n:"Slam"
tmux send-keys 'roslaunch mbzirc_husky gmapping.launch'
tmux split-window
sleep 0.2
tmux send-keys 'roslaunch mbzirc_husky pathfinding.launch'
tmux split-window -h
sleep 0.2
tmux send-keys 'roslaunch mbzirc_husky amcl.launch'
tmux split-window
sleep 0.2
tmux send-keys 'roslaunch mbzirc_husky laserscan.launch'
tmux split-window
sleep 0.2
tmux send-keys 'roslaunch mbzirc_husky mapserver.launch'
tmux select-layout tiled
tmux new-window -n:"Arm"
tmux send-keys 'roslaunch kortex_driver kortex_driver.launch start_rviz:=false start_moveit:=true'
tmux split-window -h
tmux send-keys 'roslaunch iiwa_moveit_pouring moveit_action.launch'
tmux split-window -h
sleep 0.2
tmux send-keys 'roslaunch kortex_control_manager control_manager.launch'
tmux split-window -v
sleep 0.2
tmux send-keys 'roslaunch mrs_gripper uav.launch'
tmux select-layout even-vertical
tmux new-window -n:"Bricks"
tmux send-keys 'rostopic echo /status | grep battery_voltage'
tmux split-window -h
tmux send-keys 'rosrun mbzirc_husky brickPickup'
tmux split-window -h
sleep 0.2
tmux send-keys ''
tmux split-window -h
sleep 0.2
tmux send-keys 'rosrun mbzirc_husky brickExplore'
tmux split-window -h
sleep 0.2
tmux send-keys 'rosrun mbzirc_husky brickStack'
tmux split-window
sleep 0.2
tmux send-keys 'rosrun mbzirc_husky brickDetector'
tmux split-window 
sleep 0.2
tmux send-keys 'rosrun mbzirc_husky brickStateMachine'
tmux split-window -h
sleep 0.2
tmux send-keys 'rostopic echo /kinova/arm_manager/status'
tmux split-window -h
sleep 0.2
tmux send-keys 'roslaunch mbzirc_husky inventory.launch'
tmux select-layout tiled
tmux new-window -n:"Detectors"
tmux send-keys ''
sleep 0.2
tmux split-window -h
tmux send-keys ''
sleep 0.2
tmux split-window
tmux send-keys ''
sleep 0.2
tmux split-window -h
tmux send-keys ''
sleep 0.2
tmux select-layout tiled
