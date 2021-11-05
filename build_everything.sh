#!/bin/bash

echo ""
echo "Step 1: Build the catkin_ws"
echo ""

./run_inference.sh -c "./build_catkin.sh"

echo ""
echo "Step 2: Build everything except ros1_bridge"
echo ""

./run_inference.sh -c "./build_elo_ws.sh"

echo ""
echo "Step 3: Build everything except ros1_bridge"
echo ""

./run_inference.sh -c "./build_ros1_bridge.sh"
