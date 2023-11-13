rosbag_node=$(rosnode list | grep record_); #rosnode kill $rosbag_node
if [ -z "$rosbag_node" ]
then
  tmux kill-session
else
  rosnode kill $rosbag_node
  tmux kill-session
fi
#tmux kill-session
