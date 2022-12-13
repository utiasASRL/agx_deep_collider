

# Initial image

#############

# ROS Foxy + Pytorch
# ARG BASE_IMAGE=dustynv/ros:foxy-ros-base-l4t-r35.1.0

# CUDA + TensorRT
# ARG BASE_IMAGE=nvcr.io/nvidia/l4t-tensorrt:r8.4.1.5-devel

# Pytorch
# ARG BASE_IMAGE=nvcr.io/nvidia/l4t-pytorch:r35.1.0-pth1.12-py3

# Full ML image
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-ml:r35.1.0-py3
# 	TensorFlow 1.15.5
# 	PyTorch v1.12.0
# 	torchvision v0.13.0
# 	torchaudio v0.12.0
# 	onnx 1.12.0
# 	CuPy 10.2.0
# 	numpy 1.22.4
# 	numba 0.56.0
# 	PyCUDA 2022.1
# 	OpenCV 4.5.0 (with CUDA)
# 	pandas 1.4.3
# 	scipy 1.9.0
# 	scikit-learn 1.1.1
# 	JupyterLab 3.4.4

#############



FROM ${BASE_IMAGE}

ARG ROS_PKG=desktop_full
ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/foxy

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

WORKDIR /tmp

# change the locale from POSIX to UTF-8
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        gcc \
        g++ \
        python3 \
        python3-pip \
        python3-dev \
        gedit \
        wget \
        vim \
        nano \
        git \
		cmake 

 
# 
# Install python packages
#
RUN pip3 install --upgrade pip     
RUN pip3 install --upgrade setuptools  
RUN pip3 install numpy scikit-learn matplotlib  
RUN pip3 install --upgrade psutil 
RUN pip3 install --upgrade pynvml  
RUN pip3 install --upgrade autopep8
RUN pip3 install --upgrade flake8
RUN pip3 install --upgrade imageio


#----------------------------------------------------------------------------------------------------------------------------------------------------

# 
# add the ROS deb repo to the apt sources list
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
		build-essential \
		curl \
		gnupg2 \
		lsb-release \
		ca-certificates
    #&& rm -rf /var/lib/apt/lists/*
    

#
######################
# Install ROS Noetic #
######################
#

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update 

# RUN apt-get install ros-noetic-ros-base -y --no-install-recommends
# RUN apt-get install ros-noetic-tf2* -y --no-install-recommends

#
# install bootstrap dependencies
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
          libpython3-dev \
          python3-rosdep \
          python3-rosinstall-generator \
          python3-vcstool \
          build-essential && \
    rosdep init && \
    rosdep update

ENV ROS_PKG=ros_base
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/noetic
ENV ROS_PYTHON_VERSION=3


RUN apt-get install ros-noetic-ros-base -y --no-install-recommends
RUN apt-get install ros-noetic-tf2* -y --no-install-recommends


# #
# # download/build the ROS source
# #
# RUN mkdir ros_catkin_ws && \
#     cd ros_catkin_ws && \
#     rosinstall_generator ${ROS_PKG} vision_msgs --rosdistro ${ROS_DISTRO} --deps --tar > ${ROS_DISTRO}-${ROS_PKG}.rosinstall && \
#     mkdir src && \
#     vcs import --input ${ROS_DISTRO}-${ROS_PKG}.rosinstall ./src && \
#     apt-get update


# RUN cd /tmp/ros_catkin_ws && \
# 	rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro ${ROS_DISTRO} --skip-keys python3-pykdl -y


# RUN cd /tmp/ros_catkin_ws && \
#     python3 ./src/catkin/bin/catkin_make_isolated --install --install-space ${ROS_ROOT} -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF




#
####################
# Install ROS foxy #
####################
#

ENV ROS_PKG=ros_base
ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/foxy
ENV ROS_PYTHON_VERSION=3


# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8


RUN apt-get install software-properties-common -y --no-install-recommends
RUN add-apt-repository universe

RUN apt update && sudo apt install curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update
RUN apt-get upgrade -y --no-install-recommends

RUN apt-get install ros-foxy-ros-base -y --no-install-recommends



# # 
# # Prepare dependencies
# #
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update

RUN apt-get install -y --no-install-recommends \
		python3-colcon-common-extensions 

RUN  pip3 install argcomplete \
		flake8-blind-except \
		flake8-builtins \
		flake8-class-newline \
		flake8-comprehensions \
		flake8-deprecated \
		flake8-docstrings \
		flake8-import-order \
		flake8-quotes

# #
# # Set the default DDS middleware to cyclonedds
# # https://github.com/ros2/rclcpp/issues/1335
# #
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# REinstall CMake latest version from kitware
RUN apt-get update && \
	apt-get install -y software-properties-common lsb-release
	
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"

RUN apt-get update
RUN apt-get install -y --no-install-recommends --upgrade cmake

#
###############
# Final steps #
###############
#

RUN apt-get install xterm -y --no-install-recommends
ENV TERM xterm-256color


RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update
# RUN apt-get install ros-noetic-jackal* -y --no-install-recommends
RUN apt-get install ros-noetic-move-base-flex -y --no-install-recommends
RUN apt-get install libsuitesparse-dev -y --no-install-recommends
RUN apt-get install ros-noetic-libg2o -y --no-install-recommends
RUN apt-get install ros-noetic-ros-numpy -y --no-install-recommends
RUN apt-get install ros-noetic-navigation -y --no-install-recommends

RUN apt-get update
# RUN apt-get install ros-noetic-jsk-rviz-plugins -y --no-install-recommends

RUN pip3 install  catkin_tools
RUN apt-get install ros-noetic-geodesy -y --no-install-recommends
# RUN apt-get install ros-noetic-costmap-converter -y --no-install-recommends



#----------------------------------------------------------------------------------------------------------------------------------------------------


#
# User info
# 
ARG username0=bag
ARG userid0=1085

ENV PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/cuda-11.4/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ENV LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/usr/local/cuda/lib64:/usr/local/cuda-11.3/targets/x86_64-linux/lib:


#
# User info
# 
RUN useradd -u $userid0 $username0
RUN adduser $username0 root
RUN adduser $username0 sudo 
RUN echo $username0':rosie' | chpasswd  

WORKDIR /home/$username0/eloquent_ws
RUN chown -R $username0:$username0 /home/$username0

# USER $username0  

# Set xterm defaults
WORKDIR /home/$username0
RUN echo "xterm*background: black" >> .Xresources
RUN echo "xterm*foreground: lightgray" >> .Xresources
RUN echo "xterm*faceName: monospace:pixelsize=14" >> .Xresources

CMD ["bash"]   

