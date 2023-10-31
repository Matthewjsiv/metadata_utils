rosbag_node=$(rosnode list | grep record_); rosnode kill $rosbag_node
tmux kill-session

