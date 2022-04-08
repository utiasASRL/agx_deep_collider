# Zeus
[![pipeline status](https://gitlab.com/aUToronto/zeus/badges/master/pipeline.svg)](https://gitlab.com/aUToronto/zeus/commits/master)
[![coverage report](https://gitlab.com/aUToronto/zeus/badges/master/coverage.svg)](https://gitlab.com/aUToronto/zeus/commits/master)

![Zeus](figs/zeus_year2.png "Zeus")

## Overview of Repository
This repository is being developed by aUToronto to compete in the SAE AutoDrive Challenge, a three-year collegiate competition to develop a Level 4 self-driving car. aUToronto is a team of undergraduate and graduate students at the University of Toronto.

The software provided here is intended to be run on our customized Chevrolet Bolt EV, which we call Zeus. So far, aUToronto has placed first at Year 1 and Year 2 autodrive challenges. The autonomy software and utilities contained in this repository have been heavily tested on closed courses only and have not been tested on public roads.

The second year of the competition was held in June 2019 at MCity, a mock town built for self-driving testing at the University of Michigan. Teams were required to autonomously navigate a series of intersections while handling pedestrians, traffic lights, and traffic signs. These tasks required a high degree of autonomy. Only one hour was given at the start of the competition to perform minor adjustments. Furthermore, only two attempts were allowed for each course.

The system we have designed is exceptionally reliable and computationally efficient. Our system makes use of many state-of-the-art components in order to achieve high performance while keeping the software as simple as possible. Due to Intel being an exclusive compute sponsor of the AutoDrive competition, they have restricted teams from using NVIDIA GPUs for onboard computations. As a result, we have invested significant effort into developing an autonomy stack that can run on CPUs only. Furthermore, we have developed a C++ wrapper around Intel's OpenVINO library which allows us to run Deep Neural Networks in real-time. So far, we have used this library to run SqueezeDet, YoloV3, and PointPillars on CPUs and Arria 10 FPGAs.

## Highlights

Our controller consists of a **Model Predictive Controller (MPC)** which solves a finite-time horizon optimization problem in order to pick the vehicle commands that will result in low tracking error while staying within our desired kinematic envelope (velocity, acceleration, jerk constraints).

Our traffic light detection software uses a **Deep Neural Network** (either SqueezeDet or YOLOv3) to detect and classify traffic lights within an image. These detections are associated with the expected traffic light positions contained in our semantic map. Traffic light states are smoothed temporally.

Our semantic map is based on the **OpenStreetMap format**, but tailored to our specific needs. We possess maps of several test sites: UTIAS, Oshawa, MCity, and the TRC. Each map has two components: a x.json file and an x_lattice.json file. The latter contains the lattice graph which we use to perform trajectory planning.

Our object detection system uses a **Deep Neural Network** (either SqueezeDet or YOLOv3) to detect pedestrians within an image. Euclidean clustering is used to obtain a 3D centroid for each pedestrian. A secondary clustering node is used to track other potential objects of interest. Tracking is performed using a linear Kalman filter. Object classes are smoothed temporally.

Traffic sign detection uses a combination of **Haar Cascades and Suport Vector Machines** to obtain 2D detections of signs. 3D centroids are extracted and tracked in the same manner as is done with pedestrians.

Our planner uses a **fixed-rate hierarchical structure** to handle all the perception inputs and plan safe paths. Planning is performed on a **precomputed lattice graph** of the environment which makes our planner deterministic rather than probabilistic. Obstacles, signs, and traffic lights are handled by placing stop-lines alongs the generated path as well as by cutting edges in the lattice graph. We are able to nudge around static obstacles extending into the driving lane and anticipate the motion of dynamic traffic participants.


## Folder Structure

**Autonomy Nodes**

`controller`

Contains the code for running the Model Predictive Controller.

`gps`

Contains a couple simple nodes for collecting and publishing GPS waypoints. This can be used for very simple autonomous driving demonstrations.


`lights_n_signs`

Contains our traffic light and traffic sign detection code.

`object_detection`

Contains the primary 2D-3D clustering node named cluster\_publisher, and the asynchronous tracker named object_tracker. Also contains a secondary clustering node that performs Euclidean clustering on the entire pointcloud, and a node for assessing the occupancy of parking spots.

`planner`

Contains the software for processing inputs from the perception and generating safe paths. The output is a series of (x,y) positions and a desired velocity along the path.

**Libraries**

`map`

This library contains many utility functions related to our semantic map which are used throughout our repository.

`kalman`

This library contains a general purpose class for tracking objects in a static map frame where the detections are output in a moving sensor frame (c2).

`zeus_pcl`

This library contains many functions that relate to processing and working with LIDAR pointclouds.

**Utility Software**

`dla`

This folder contains our ROS package vino_ros which is a wrapper around the OpenVINO deep learning acceleration library.

`vehicle_interface/`

`vi_ros`

This software allows us to communicate with our Chevrolet Bolt and send torque, steering, transmission, and other miscellaneous commands.

The following components of the aUToronto system are not in this repository and need to be cloned separately under `~/catkin_ws/src/` :

**Software**
1. Point Grey Blackfly Camera Driver [Repository](https://gitlab.com/aUToronto/Drivers/flir_camera_driver)
2. Velodyne LIDAR Driver [Repository](https://gitlab.com/aUToronto/software/velodyne/-/tree/autoronto)
3. Novatel Driver [Repository](https://gitlab.com/aUToronto/Drivers/novatel_span_driver)

## Software Architecture

![Software Architecture](figs/sw_arch.png "Software Architecture")

## Sensor Positions

![Zeus](figs/zeus.png "Zeus")
![FOV](figs/fov.png "FOV")
![Side View](figs/side-view.png "Side View")
![Top View](figs/top-view.png "Top View")

## How to Build Zeus


## Installation and Requirements

From a fresh machine (Ubuntu 16.0.4 LTS + ROS Kinetic Desktop Full installed) running the following to build `zeus`:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://gitlab.com/aUToronto/zeus --recursive
git clone https://gitlab.com/aUToronto/software/velodyne
cd velodyne
git checkout autoronto
cd ..
./zeus/build-tools/install_dependencies.sh
cd ~/catkin_ws
catkin init
catkin build -DCMAKE_BUILD_TYPE=Release -DUSE_ZEUS_MESSAGES=ON
```

NOTE: If you don't want DLA (it is currently very large: ~500 MB, remove the `--recursive` from the clone command).
You can pull dla after zeus by doing:
```
cd dla
git submodule init
git submodule update
```

NOTE: Some DNN weights are not tracked in git since they are too big (100s of MBs), they can be downloaded with the
following script:

```
cd ~/catkin_ws/src/
./zeus/build-tools/download_models.sh
```

NOTE: the first argument to `./configure.sh` is the location of the `zeus` repo (in this case it is relatively `.`) and the second argument is the location in which to place the catkin workspace (Chosen to be `../build`, but note that the path must then be changed in step 5; moreover, the path cannot be inside of the `zeus` repo).
The `./configure.sh` step only has to be done once on each machine, from then on the workspace is stable and everything needed can happen through the catkin workspace (as long as the environment sourced in step 4 is active).

## Running Zeus (Online)

Terminal 1 (sensor drivers):
```
cd ~/catkin_ws
source devel/setup.bash
roscore &
roslaunch zeus sensors.launch
```

Terminal 2 (runs autonomy stack):
```
cd ~/merge_ws
source devel/setup.bash
source src/zeus/dla/zeus_inference_engine/setupvars.sh
roslaunch zeus autonomy.launch
```

`autonomy.launch` contains all the high-level arguments you might need to change depending on the desired functionality. For example, `map_name` can be set to `mcity` or `oshawa`. For live operation, `simulation` should be set to `false`. We include a boolean argument to determine whether or not to run each major autonomy node such as `objects` for object detection and `lights` for traffic light detection.

`record.launch` uses a hard-coded path to specify where the recorded rosbag should be saved to. Change this or disable recording by setting `record` to `false`.

Zeus currently uses four cameras (locations and numbering shown above). `autonomy.launch` also contains arguments for enabling or disabling these cameras. Note that traffic light detection, sign detection, and object detection are set up to use more than one camera if they are available. If the computer you're using is not that powerful, consider disabling all but camera 1.

Terminal 3 (vehicle interface):
```
cd ~/merge_ws
source devel/setup.bash
rosrun vi_ros vi_ros_node
```

The vehicle interface will ask the operator to cin something and then press `enter`. Once you do so, the vehicle will begin to drive autonomously. ***Be extra careful to ensure that there are no pedestrians or other hazards close to the vehicle before entering autonomous mode.***

## Running Zeus (Offline)

Terminal 1 (sensor drivers):
```
cd ~/catkin_ws
source devel/setup.bash
roscore &
source src/zeus/dla/zeus_inference_engine/setupvars.sh
roslaunch zeus playback.launch
```

`playback.launch` will launch the desired autonomy nodes with `use_sim_time` set to `true`. It also contains a couple helpful nodes for uncompressing images from a rosbag and converting `velodyne_packets` to `velodyne_points`. Note that this node contains a hard-coded path to a rosbag as an example, change this to the rosbag you would like to run. Special attention to be paid to the `--clock` flag added to the rosbag command. This publishes the simulated ros time.

You can find rosbags from the year 2 competition [here](https://drive.google.com/open?id=1ZkC0cQ9n92Qr3u5ySJzZBG46DU408cNb).

## Publications

If you found this repository was helpful in your work, please consider citing one or more of the following papers:

[Building a Winning Self-Driving Car in Six Months](https://arxiv.org/abs/1811.01273)

```
@inproceedings{burnett2019building,
  title={Building a winning self-driving car in six months},
  author={Burnett, Keenan and Schimpe, Andreas and Samavi, Sepehr and Gridseth, Mona and Liu, Chengzhi Winston and Li, Qiyang and Kroeze, Zachary and Schoellig, Angela P},
  booktitle={2019 International Conference on Robotics and Automation (ICRA)},
  pages={9583--9589},
  year={2019},
  organization={IEEE}
}
```

[aUToTrack: A Lightweight Object Detection and Tracking System for the SAE AutoDrive Challenge](https://arxiv.org/abs/1905.08758)

```
@inproceedings{burnett2019autotrack,
  title={aUToTrack: A Lightweight Object Detection and Tracking System for the SAE AutoDrive Challenge},
  author={Burnett, Keenan and Samavi, Sepehr and Waslander, Steven and Barfoot, Timothy and Schoellig, Angela},
  booktitle={2019 16th Conference on Computer and Robot Vision (CRV)},
  pages={209--216},
  year={2019},
  organization={IEEE}
}
```

[Zeus: A System Description of the Two-Time Winner of the Collegiate SAE AutoDrive Competition](https://arxiv.org/abs/2004.08752)

```
@article{doi:10.1002/rob.21958,
    author = {Burnett, Keenan and Qian, Jingxing and Du, Xintong and Liu, Linqiao and Yoon, David J. and Shen, Tianchang and Sun, Susan and Samavi, Sepehr and Sorocky, Michael J. and Bianchi, Mollie and Zhang, Kaicheng and Arkhangorodsky, Arkady and Sykora, Quinlan and Lu, Shichen and Huang, Yizhou and Schoellig, Angela P. and Barfoot, Timothy D.},
    title = {Zeus: A system description of the two-time winner of the collegiate SAE autodrive competition},
    journal = {Journal of Field Robotics},m
}
```

## Pre-Flight Checks
1. Check Sensors
    1. Verify cameras are publishing, check their publishing speed (should be >= 10 Hz)
    2. In RQT, verify the cameras are not dirty, or are not too dark
    3. rostopic echo /navsat/odom and /imu/data (verify that the position, velocity, and acceleration data looks good)
    4. Use either the Novatel console or the Applanix GUI to verify that the GPS/IMU position is converged.
    5. Verify velodyne is publishing in rviz
2. Waypoints / validate global path is correct. Look at the planned path in rviz, verify it is correct and does not pass through any obstacles.
3. Verify the correct autonomy nodes are running
    1. Verify publishing rate of perception topics are as expected
    2. Check rqt visualization where applicable
4. Check the desired speed, acceleration limits within `planner.launch`

## Common Problems / Remedial Actions
1. RVIZ does not display map as expected
    1. Check that correct map is being used.
    2. Check that custom_rviz is running properly
    3. Kill rviz, relaunch it
    4. Check that the TF tree is being published (`bolt_tf.launch`)
    5. Check that `use_sim_time` is set to the desired value
2. Vehicle interface fails to enter autonomous mode or propulsion does not work
    1. Check that the autonomous/manual switch is in the autonomous position.
    2. Check the cable connections for CAN communication
    3. If you see persistent errors, restart the Intel server (very rare).
    4. If restarting Intel server does not fix the issue, there may be some error codes in the vehicle that need to be cleared. Use the OBDLink tool to clear all the errors (occurs after e-stop usage).
