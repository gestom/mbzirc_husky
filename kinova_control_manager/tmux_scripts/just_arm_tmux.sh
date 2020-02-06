tmux send-keys 'rostopic echo /status | grep battery_voltage'
tmux split-window -h
tmux send-keys 'roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300'
tmux split-window -v
tmux select-layout tiled
sleep 0.2
tmux send-keys 'roslaunch kinova_control_manager control_manager.launch'
tmux split-window -h
tmux send-keys 'roslaunch realsense2_camera rs_rgbd.launch'
tmux select-layout tiled
sleep 0.2
tmux split-window -h
tmux send-keys 'rosrun mbzirc_husky brickDetector'
tmux split-window -v
tmux send-keys 'roslaunch mrs_gripper uav.launch'
tmux select-layout tiled
sleep 0.2
tmux split-window -h
tmux send-keys 'rosservice call /kinova/arm_manager'
tmux split-window -v
tmux send-keys 'rostopic echo /kinova/arm_manager/status'
sleep 0.2
tmux select-layout tiled
