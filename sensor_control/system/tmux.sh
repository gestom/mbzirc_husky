tmux rename-window -t 0 "Devices"
tmux send-keys 'roslaunch mbzirc_husky robot.launch'
tmux split-window -h
sleep 0.2
tmux send-keys 'roslaunch mbzirc_husky velodyne.launch'
tmux split-window 
sleep 0.2
tmux send-keys 'roslaunch mbzirc_husky arm.launch'
tmux split-window 
sleep 0.2
tmux send-keys 'roslaunch mbzirc_husky pump.launch'
tmux split-window
sleep 0.2
tmux send-keys 'roslaunch mbzirc_husky realsense.launch'
tmux new-window -n:"Slam"
tmux send-keys 'roslaunch mbzirc_husky gmapping.launch'
