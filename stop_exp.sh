#!/bin/bash


# Global source
source "/opt/ros/noetic/setup.bash"
export ROS_HOSTNAME=cpr-tor59-xav02
export ROS_MASTER_URI=http://cpr-tor59-02:11311

# local ws source
source "catkin_ws/devel/setup.bash"

# First kill move_base in case of emergency
rosnode kill move_base

all_nodes=""
for node_name in "frame_update" "record_" "follow_waypoints"
do
    node=$(rosnode list | grep "$node_name")
    if [ -n "$node" ]; then
        if [ ${node:0:1} = "/" ]; then
            all_nodes="$all_nodes $node"
        fi
    fi
done

if [ -n "$all_nodes" ]; then
    rosnode kill $all_nodes
else
    echo "No experiment node running"
fi


# # Additionnaly, kill nodes on Xavier board
# ssh -i $HOME/.ssh/id_rsa administrator@cpr-tor59-xav01 "cd catkin_ws/scripts/ && ./stop_exp.sh"
