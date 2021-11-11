#
# this dockerfile roughly follows the 'Ubuntu install of ROS Melodic' from:
#   http://wiki.ros.org/melodic/Installation/Ubuntu
#
ARG BASE_IMAGE=nvcr.io/nvidia/l4t-pytorch:r32.4.3-pth1.6-py3
FROM ${BASE_IMAGE}

ARG ROS_PKG=desktop_full
ENV ROS_DISTRO=foxy
ENV ROS_ROOT=/opt/ros/foxy

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
####################
# Install ROS foxy #
####################
#

# 
# Prepare dependencies
#
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update

RUN apt-get install -y --no-install-recommends python3-catkin-pkg
RUN apt-get install -y --no-install-recommends python3-rospkg
RUN apt-get install -y --no-install-recommends python3-rosdistro
RUN apt-get install -y --no-install-recommends python3-rosdep-modules

RUN apt-get install -y --no-install-recommends \
		libbullet-dev \
		libpython3-dev \
		python3-colcon-common-extensions \
		python3-flake8 \
		python3-pip \
		python3-numpy \
		python3-pytest-cov \
		python3-rosdep \
		python3-setuptools \
		python3-vcstool \
		python3-rosinstall-generator

RUN apt-get install -y --no-install-recommends \
		libasio-dev \
		libtinyxml2-dev \
		libcunit1-dev \
		libgazebo9-dev \
		gazebo9 \
		gazebo9-common \
		gazebo9-plugin-base

RUN  pip3 install argcomplete \
		flake8-blind-except \
		flake8-builtins \
		flake8-class-newline \
		flake8-comprehensions \
		flake8-deprecated \
		flake8-docstrings \
		flake8-import-order \
		flake8-quotes \
		pytest-repeat \
		pytest-rerunfailures \
		pytest
	
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

# 
# download/build ROS from source
#
ARG ROS_PKG=ros_base
ENV ROS_PKG=ros_base
# https://answers.ros.org/question/325245/minimal-ros2-installation/?answer=325249#post-id-325249
RUN mkdir -p ${ROS_ROOT}/src

RUN cd ${ROS_ROOT} && \
    rosinstall_generator --deps --rosdistro ${ROS_DISTRO} ${ROS_PKG} \
	launch_xml \
	launch_yaml \
	example_interfaces > ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall

#
# Split the rosinstall file to make sure there in no bug with network
#
RUN cd ${ROS_ROOT} && \
	split -l 4 ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall seg_ --verbose

RUN cd ${ROS_ROOT} && \
	for f in seg_*; do echo $f && if [[ $(wc -l < $f) -ge 4 ]]; then echo "OK"; fi; done

#
# Install each splitted install file
#
RUN cd ${ROS_ROOT} && \
	for f in seg_a*; do if [[ $(wc -l < $f) -ge 4 ]]; then cat $f && vcs import --debug src < $f; fi; done
RUN cd ${ROS_ROOT} && \
	for f in seg_b*; do if [[ $(wc -l < $f) -ge 4 ]]; then cat $f && vcs import --debug src < $f; fi; done
RUN cd ${ROS_ROOT} && \
	for f in seg_c*; do if [[ $(wc -l < $f) -ge 4 ]]; then cat $f && vcs import --debug src < $f; fi; done
RUN cd ${ROS_ROOT} && \
	for f in seg_d*; do if [[ $(wc -l < $f) -ge 4 ]]; then cat $f && vcs import --debug src < $f; fi; done
RUN cd ${ROS_ROOT} && \
	for f in seg_e*; do if [[ $(wc -l < $f) -ge 4 ]]; then cat $f && vcs import --debug src < $f; fi; done
RUN cd ${ROS_ROOT} && \
	for f in seg_f*; do if [[ $(wc -l < $f) -ge 4 ]]; then cat $f && vcs import --debug src < $f; fi; done
RUN echo "1"
RUN cd ${ROS_ROOT} && \
	for f in seg_g*; do if [[ $(wc -l < $f) -ge 4 ]]; then cat $f && vcs import --debug src < $f; fi; done
RUN cd ${ROS_ROOT} && \
	for f in seg_h*; do if [[ $(wc -l < $f) -ge 4 ]]; then cat $f && vcs import --debug src < $f; fi; done

#
# download unreleased packages
#  
RUN git clone --branch ros2 https://github.com/Kukanani/vision_msgs ${ROS_ROOT}/src/vision_msgs && \
	#git clone --branch debian/foxy/gmock_vendor https://github.com/ros2-gbp/googletest-release ${ROS_ROOT}/src/googletest/gmock_vendor && \
    git clone --branch ${ROS_DISTRO} https://github.com/ros2/demos demos && \
    cp -r demos/demo_nodes_cpp ${ROS_ROOT}/src && \
    cp -r demos/demo_nodes_py ${ROS_ROOT}/src && \
    rm -r -f demos
	
# patch libyaml - https://github.com/dusty-nv/jetson-containers/issues/41#issuecomment-774767272
RUN cd ${ROS_ROOT} && \
	rm ${ROS_ROOT}/src/libyaml_vendor/CMakeLists.txt && \
	wget --no-check-certificate https://raw.githubusercontent.com/ros2/libyaml_vendor/master/CMakeLists.txt -P ${ROS_ROOT}/src/libyaml_vendor/

# libeigen3-dev libspdlog-dev python3-lark-parser python3-pygments uncrustify libxslt1.1 python3-bs4 python3-chardet python3-html5lib python3-lxml python3-webencodings
# python3-asn1crypto python3-cffi-backend python3-cryptography python3-idna net-tools python3-ifcfg python3-mock python3-pbr libbenchmark-dev libbenchmark1
# python3-netifaces libxml2-utils libzstd-dev pydocstyle python3-pydocstyle python3-snowballstemmer

# ca-certificates-java default-jdk default-jdk-headless default-jre
# default-jre-headless fonts-dejavu-extra java-common libatk-wrapper-java
# libatk-wrapper-java-jni libfontenc1 libxft2 libxmuu1 libxtst6 libxxf86dga1
# openjdk-11-jdk openjdk-11-jdk-headless openjdk-11-jre
# openjdk-11-jre-headless x11-utils

# libaopalliance-java libapache-pom-java libatinject-jsr330-api-java
# libcdi-api-java libcommons-cli-java libcommons-io-java libcommons-lang3-java
# libcommons-parent-java libgeronimo-annotation-1.3-spec-java
# libgeronimo-interceptor-3.0-spec-java libguava-java libguice-java
# libhawtjni-runtime-java libjansi-java libjansi-native-java libjsr305-java
# libmaven-parent-java libmaven-resolver-java libmaven-shared-utils-java
# libmaven3-core-java libplexus-cipher-java libplexus-classworlds-java
# libplexus-component-annotations-java libplexus-interpolation-java
# libplexus-sec-dispatcher-java libplexus-utils2-java libsisu-inject-java
# libsisu-plexus-java libslf4j-java libwagon-file-java
# libwagon-http-shaded-java libwagon-provider-api-java maven

# python3-pytest-mock libcppunit-1.14-0 libcppunit-dev



#
# install dependencies using rosdep
#
RUN apt-get update && \
    cd ${ROS_ROOT} && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
		--skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers qt_gui ros2interface kdl_parser gmock_vendor"
    
#
# build it!
#
RUN cd ${ROS_ROOT} && \
	colcon build --symlink-install

#
# Set the default DDS middleware to cyclonedds
# https://github.com/ros2/rclcpp/issues/1335
#
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#
###############
# Final steps #
###############
#

#
# Additional ros packages
#
# RUN apt-get install ros-foxy-velodyne* -y --no-install-recommends
# RUN apt-get install ros-eloquent-pointcloud-to-laserscan -y --no-install-recommends
# RUN apt-get install ros-eloquent-tf2* -y --no-install-recommends
# RUN apt-get install ros-eloquent-navigation* -y --no-install-recommends
# RUN apt-get install ros-eloquent-teleop-twist-keyboard -y --no-install-recommends
# RUN apt-get install ros-eloquent-costmap-converter -y --no-install-recommends
# RUN apt-get install ros-eloquent-rqt-graph -y --no-install-recommends

RUN apt-get update     
RUN apt-get install vim gedit -y --no-install-recommends

# 
# Install ROS melodic
#
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'   
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -    
RUN apt-get update 
RUN apt-get install ros-melodic-ros-base -y --no-install-recommends     
# RUN echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc   


#
# User info
# 
ARG username0=bag    
ARG userid0=1085     
ENV PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/cuda-10.2/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin    
ENV LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/usr/local/cuda/lib64:/usr/local/cuda-10.2/targets/aarch64-linux/lib:    
ENV USER=administrator   
#COPY file:3ece9477fa538f3b109134f0cbe8f8352265fbac337135de2873d02de1d32f13 in /ros_entrypoint.sh    
# COPY ros_entrypoint.sh /ros_entrypoint.sh


#
# Additional packages
#
# RUN apt-get install ros-eloquent-costmap-converter-msgs -y --no-install-recommends
RUN apt-get install nano -y --no-install-recommends
RUN pip3 install --upgrade scikit-learn
RUN pip3 install --upgrade imageio

# RUN apt-get install ros-eloquent-ros1-bridge -y --no-install-recommends

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

