#
# this dockerfile roughly follows the 'Ubuntu install of ROS Melodic' from:
#   http://wiki.ros.org/melodic/Installation/Ubuntu
#
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-pytorch:r32.4.3-pth1.6-py3
FROM ${BASE_IMAGE}

ARG ROS_PKG=desktop_full
ENV ROS_DISTRO=eloquent
ENV ROS_ROOT=/opt/ros/eloquent

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

WORKDIR /tmp

# change the locale from POSIX to UTF-8
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV PYTHONIOENCODING=utf-8

 
# 
# Install python packages
#
RUN pip3 install --upgrade pip     
RUN pip3 install setuptools  
RUN pip3 install numpy scikit-learn matplotlib PyYAML  
RUN pip3 install psutil 
RUN pip3 install pynvml  
RUN pip3 install autopep8
RUN pip3 install flake8


# 
# add the ROS deb repo to the apt sources list
#
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release \
		ca-certificates
    #&& rm -rf /var/lib/apt/lists/*
    

# 
# Install ROS melodic
#
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'   
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -    
RUN apt-get update 
RUN apt-get install ros-melodic-ros-base -y --no-install-recommends     
RUN echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc    


# 
# Install ROS eloquent
#
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -    
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
ARG ROS_PKG=desktop   
RUN apt-get update  

RUN apt-get install -y --no-install-recommends \
		ros-eloquent-`echo "${ROS_PKG}" | tr '_' '-'` \
		ros-eloquent-launch-xml \
		ros-eloquent-launch-yaml \
		ros-eloquent-launch-testing \
		ros-eloquent-launch-testing-ament-cmake \
		ros-eloquent-camera-calibration-parsers \
		ros-eloquent-camera-info-manager \
		ros-eloquent-cv-bridge \
		ros-eloquent-v4l2-camera \
		ros-eloquent-vision-msgs \
		ros-eloquent-vision-opencv \
		ros-eloquent-image-transport \
		ros-eloquent-image-tools \
		ros-eloquent-image-geometry \
		ros-eloquent-gazebo-ros \
		ros-eloquent-gazebo-msgs \
		ros-eloquent-gazebo-ros-pkgs \
		ros-eloquent-gazebo-plugins

###############

# RUN apt-get install -y --no-install-recommends \
#         python3-catkin-pkg \
#         python3-rosdistro \
#         python3-rospkg \
#         python3-rosdep-modules

# RUN apt-get install -y --no-install-recommends \t\tpython3-catkin-pkg \t\tpython3-rosdistro \t\tpython3-rospkg \t\tpython3-rosdep-modules

RUN apt-get install -y --no-install-recommends \
		python3-catkin-pkg \
		python3-rosdistro \
		python3-rospkg \
		python3-rosdep-modules



RUN apt-get install -y --no-install-recommends \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-rosdep \
		libgazebo9-dev \
		gazebo9 \
		gazebo9-common \
		gazebo9-plugin-base


# 
# init/update rosdep
#
RUN	apt-get update && \
    cd ${ROS_ROOT} && \
    rosdep init && \
    rosdep update


#
# Additional ros packages
#
RUN apt-get install ros-eloquent-velodyne* -y --no-install-recommends
RUN apt-get install ros-eloquent-pointcloud-to-laserscan -y --no-install-recommends
RUN apt-get install ros-eloquent-tf2* -y --no-install-recommends
RUN apt-get install ros-eloquent-navigation* -y --no-install-recommends
RUN apt-get install ros-eloquent-teleop-twist-keyboard -y --no-install-recommends
RUN apt-get install ros-eloquent-costmap-converter -y --no-install-recommends
RUN apt-get install ros-eloquent-rqt-graph -y --no-install-recommends


#
# compile yaml-cpp-0.6, which some ROS packages may use (but is not in the 18.04 apt repo)
#
RUN git clone --branch yaml-cpp-0.6.0 https://github.com/jbeder/yaml-cpp yaml-cpp-0.6 && \
    cd yaml-cpp-0.6 && \
    mkdir build && \
    cd build && \
    cmake -DBUILD_SHARED_LIBS=ON .. && \
    make -j$(nproc) && \
    cp libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/ && \
    ln -s /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6.0 /usr/lib/aarch64-linux-gnu/libyaml-cpp.so.0.6
    
RUN apt-get update     
RUN apt-get install vim gedit -y --no-install-recommends     


#
# User info
# 
ARG username0=bag    
ARG userid0=1085     
ENV PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/cuda-10.2/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin    
ENV LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/usr/local/cuda/lib64:/usr/local/cuda-10.2/targets/aarch64-linux/lib:    
ENV USER=administrator   
#COPY file:3ece9477fa538f3b109134f0cbe8f8352265fbac337135de2873d02de1d32f13 in /ros_entrypoint.sh    
COPY ros_entrypoint.sh /ros_entrypoint.sh


#
# Additional packages
#
RUN apt-get install ros-eloquent-costmap-converter-msgs -y --no-install-recommends
RUN apt-get install ros-eloquent-ros1-bridge -y --no-install-recommends
RUN apt-get install nano -y --no-install-recommends
RUN pip3 install --upgrade scikit-learn
RUN pip3 install --upgrade imageio


#
# User info
# 
RUN useradd -ms /bin/bash -u $userid0 $username0
RUN adduser $username0 root
RUN adduser $username0 sudo 
RUN echo $username0':myhaljack' | chpasswd  
USER administrator  



CMD ["bash"]   
WORKDIR /home/administrator/eloquent_ws   

