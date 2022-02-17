#!/bin/bash

########
# Init #
########

echo ""
echo "Running ros-eloquent docker."
echo ""

detach=false
devdoc=false
asroot=false
command=""

while getopts dvrc: option
do
case "${option}"
in
d) detach=true;;
v) devdoc=true;;
r) asroot=true;;
c) command=${OPTARG};;
esac
done

if [ -n "$command" ]; then
  echo -e "Running command $command\n"
fi

###########################
# Start docker containers #
###########################

# Docker run arguments (depending if we run detached or not)
docker_args="-it --rm --gpus all --shm-size=64g" # --runtime nvidia"

# Running detached
if [ "$detach" = true ] ; then
    docker_args="-d ${docker_args}"
fi

# Running as root
if [ "$asroot" = true ] ; then
    docker_args="-u root ${docker_args}"
fi


# Volumes (modify with your own path here)
volumes="-v $PWD/eloquent_ws:/home/$USER/eloquent_ws \
-v $PWD/catkin_ws:/home/$USER/catkin_ws \
-v $PWD/bridge_ws:/home/$USER/bridge_ws \
-v $PWD/results:/home/$USER/results \
-v $HOME/.ssh:/home/$USER/.ssh \
-v $PWD/Data:/home/$USER/Data"

# Additional arguments to be able to open GUI
XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
other_args="-v $XSOCK:$XSOCK \
    -v $XAUTH:$XAUTH \
    --net=host \
    --privileged \
	-e XAUTHORITY=${XAUTH} \
    -e DISPLAY=$DISPLAY \
    -e ROS_MASTER_URI=http://cpr-tor59-01:11311 \
    -w=/home/$USER/eloquent_ws/scripts"


if [ "$devdoc" = true ] ; then

    # Execute the command in docker (Example of command: ./master.sh -ve -m 2 -p Sc1_params -t A_tour)
    docker run $docker_args \
    $volumes \
    $other_args \
    --name "collidoc-dev" \
    laptop_foxy \
    $command

else

    # Current Date
    now=`date +%Y-%m-%d_%H-%M-%S`

    # Execute the command in docker (Example of command: ./master.sh -ve -m 2 -p Sc1_params -t A_tour)
    docker run $docker_args \
    $volumes \
    $other_args \
    --name "collidoc-$now" \
    laptop_foxy \
    $command

    # Attach a log parameters and log the detached docker
    if [ "$detach" = true ] ; then
        docker logs -f "$USER-collider-$now" &> $PWD/results/log_"$now".txt &
    fi

fi