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
echo "| Step 3: Build everything except ros1_bridge |"
echo "0---------------------------------------------0"

./run_inference.sh -c "./build_ros1_bridge.sh"
