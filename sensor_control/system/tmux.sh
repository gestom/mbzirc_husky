tmux rename-window -t 0 "Devices"
tmux send-keys 'rostopic echo /status | grep battery_voltage'
tmux split-window -v
tmux send-keys 'roslaunch mbzirc_husky robot.launch'
tmux split-window -h
sleep 0.2
tmux send-keys 'roslaunch mbzirc_husky velodyne.launch'
tmux split-window 
sleep 0.2
tmux send-keys 'roslaunch realsense2_camera rs_rgbd.launch'
tmux new-window -n:"Slam"
tmux send-keys 'roslaunch mbzirc_husky gmapping.launch'
tmux split-window
tmux send-keys 'roslaunch mbzirc_husky amcl.launch'
tmux split-window -h
tmux send-keys 'rosrun amcl amcl'
tmux split-window
tmux send-keys 'roslaunch mbzirc_husky laserscan.launch'
tmux split-window
tmux send-keys 'roscd mbzirc_husky;rosrun map_server map_server maps/map.yaml'
tmux select-layout tiled
tmux new-window -n:"Arm"
tmux send-keys 'roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300'
tmux split-window -v
sleep 0.2
tmux send-keys 'roslaunch kinova_control_manager control_manager.launch'
tmux split-window 
sleep 0.2
tmux send-keys 'roslaunch mrs_gripper uav.launch'
tmux select-layout even-vertical
tmux new-window -n:"Bricks"
tmux send-keys 'rostopic echo /status | grep battery_voltage'
tmux split-window -h
tmux send-keys 'rosrun mbzirc_husky brickPickup'
tmux split-window -h
sleep 0.2
tmux send-keys 'rosrun mbzirc_husky brickRearrange'
tmux split-window -h
sleep 0.2
tmux send-keys 'rosrun mbzirc_husky brickExplore'
tmux split-window -h
sleep 0.2
tmux send-keys 'rosrun mbzirc_husky brickStack'
tmux split-window -h
sleep 0.2
tmux send-keys 'rosrun mbzirc_husky brickDetector'
tmux split-window 
sleep 0.2
tmux send-keys 'rosrun mbzirc_husky brickStateMachine'
tmux split-window -h
tmux send-keys 'rostopic echo /kinova/arm_manager/status'
tmux select-layout tiled
