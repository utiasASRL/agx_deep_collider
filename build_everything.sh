#!/bin/bash

echo ""
echo ""
echo "0-----------------------------0"
echo "| Step 1: Build the catkin_ws |"
echo "0-----------------------------0"

./run_inference.sh -c "./build_catkin.sh"

echo ""
echo ""
echo "0---------------------------------------------0"
echo "| Step 2: Build everything except ros1_bridge |"
echo "0---------------------------------------------0"

./run_inference.sh -c "./build_elo_ws.sh"

echo ""
echo ""
echo "0---------------------------------------------0"
echo "| Step 3:         Build ros1_bridge           |"
echo "0---------------------------------------------0"

./run_inference.sh -c "./build_ros1_bridge.sh"



####################################################################################################################


#                                                       HERE                                                       #

# OK we should be here
# Verify output of ros1bridge compilation
# Then Compile the reste 
# Then test robot without laptop
# (note ssh command to start velodyne should work even from the same computer!)


####################################################################################################################







echo ""
echo ""
echo "0---------------------------------------------0"
echo "| Step 4:    Check the custom bridged msg     |"
echo "0---------------------------------------------0"

./run_inference.sh -c "./check_custom_bridge_msgs.sh"

echo ""
echo ""
echo "0---------------------------------------------0"
echo "| Step 5:      Compile deep c++ wrappers      |"
echo "0---------------------------------------------0"

./run_inference.sh -c "./compile_wrappers.sh"