#!/bin/bash

username=$USER
userid=$UID

echo ""
echo "Building image docker_ros_foxy for user"
echo ""

docker image build --build-arg username0=$username \
--build-arg userid0=$userid \
--shm-size=64g -t \
orin_foxy .