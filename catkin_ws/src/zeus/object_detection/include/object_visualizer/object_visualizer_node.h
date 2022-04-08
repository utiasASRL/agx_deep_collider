// Author: Keenan Burnett
// Copyright (C) 2020 aUToronto
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef OBJECT_VISUALIZER_OBJECT_VISUALIZER_NODE_H
#define OBJECT_VISUALIZER_OBJECT_VISUALIZER_NODE_H
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <zeus_msgs/BoundingBox3D.h>
#include <zeus_msgs/Detections3D.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>
#include <chrono>  // NOLINT [build/c++11]
#include <string>
#include <vector>
#include <map>
// #include "types/Pose2.hpp"
// #include "types/RoadMap.hpp"
// #include "types/Road.hpp"
// #include "types/geometry_types.hpp"
#include "types/zeus_pcl_types.hpp"
#include "utils/transform_utils.hpp"

//* ObjectVisualizerNode
/**
* \brief This ROS node receives 3D object detections (which may be the output of a tracker), as well as the
* corresponding LIDAR pointcloud, camera image, and Odometry message. This node outputs two visualizations of objects,
* one in the perspective view and one in the Bird's Eye View (BEV).
*
*/
class ObjectVisualizerNode {
 public:
    ObjectVisualizerNode(ros::Publisher persp_pub, ros::Publisher bev_pub, ros::NodeHandle nh) :
        persp_pub_(persp_pub), bev_pub_(bev_pub), nh_(nh) {}
    void set_camera_matrix(Eigen::Matrix4f P_) {P = P_;}
    void set_node_name() {node_name = ros::this_node::getName();}

    /*!
       \brief For each 3D detection, this callback will draw a 2D box on the BEV and a 3D box on the perspective image.
       In addition, the velocity vector will be visualized in the BEV as a blue arrow.
       Boxes are color-coded based on their confidence. Objects below the desired threshold are drawn orange, and
       objects above the threshold are drawn as green.
       \param det zeus_msg representing a list of 3D detections (in the odom frame)
       \param scan A 3D pointcloud output by a velodyne LIDAR.
       \param img An image output by a camera.
       \param odom The odometry (position and orientation) at the current time step.
    */
    void callback(const zeus_msgs::Detections3D::ConstPtr& det, const sensor_msgs::PointCloud2ConstPtr& scan);

    /*!
       \brief Initializes the static transforms so that they only need to be queried once.
       \pre Make sure to run get_ros_parameters() before this.
    */
    void initialize_transforms();

    /*!
       \brief Retrieve rosparams that apply to this node.
       \pre Make sure to run set_node_name() before this function.
    */
    void get_ros_parameters();

    /*!
       \brief Initializes roads and road_polygons from the road_map
    */
    void initialize_roads();

 private:
    // map::RoadMap road_map_;
    ros::Publisher persp_pub_;
    ros::Publisher bev_pub_;
    ros::NodeHandle nh_;
    zeus_tf::tfBufferPtr tfBuffer;
    zeus_tf::tfListenerPtr tfListener;
    std::string node_name = "object_visualizer";
    Eigen::Matrix4d Tvm = Eigen::Matrix4d::Identity();      /*!< Transform from velodyne to camera_frame */
    Eigen::Matrix4d Tvc = Eigen::Matrix4d::Identity();      /*!< Transform from camera_frame to velodyne */
    Eigen::Matrix4d T_bev_v = Eigen::Matrix4d::Zero();      /*!< Transform from velodyne to bev */
    Eigen::Matrix4d Tic2 = Eigen::Matrix4d::Identity();     /*!< Transform from c2 to imu_link */
    Eigen::Matrix4d Tvi = Eigen::Matrix4d::Identity();      /*!< Transform from imu_link to velodyne */
    Eigen::Matrix4d Tcc2 = Eigen::Matrix4d::Identity();     /*!< Transform from c2 to camera_frame */
    Eigen::Matrix3d R3_c2 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R4_c2 = Eigen::Matrix3d::Identity();
    Eigen::Matrix4f P = Eigen::Matrix4f::Identity();        /*!< Camera projection matrix */
    cv::Point cam_center;                                   /*!< position of camera in BEV image */
    std::vector<double> point_cloud_range = {1.1, 40.0, -14.5, 14.5, -1.8, 3.5};
    float h_metric = fabs(point_cloud_range[1] - point_cloud_range[0]);     /*!< Height of BEV image */
    float w_metric = fabs(point_cloud_range[3] - point_cloud_range[2]);     /*!< Width of BEV image */
    float max_distance_ransac = 0.25;       /*!< Points closer than this are an inlier to the RANSAC ground plane */
    int max_iterations_ransac = 25;         /*!< Maximum number of iterations to run RANSAC */
    float meter_to_pixel = 20.0;            /*!< Conversion from metric to pixels in BEV image */
    bool show_raw_dets = true;
    bool show_points = true;
    bool show_cubes = true;
    bool show_map = true;
    bool show_detailed_text = false;
    float viz_divide = 2.0;                 /*!< Size of perspective output = (H / viz_divide, W / viz_divide) */
    std::string camera_frame = "mono_link";
    int64_t road_id = 0;
    // Model Matrices for tracking ground plane parameters
    Eigen::Matrix4f K = Eigen::Matrix4f::Identity();
    Eigen::Vector4f gp_params = Eigen::Vector4f::Zero();
    // std::map<int64_t, map::polygon_type> road_polygons;
    // std::map<int64_t, map::Road> roads;
    std::vector<std::string> type_names = {"Car", "Pedestrian", "Cyclist", "Unknown"};
    int unknown_type = 3;

    /*!
       \brief This function draws the detections on the perspective and BEV visualizations.
    */
    void show_tracked_dets(cv::Mat &bev, Eigen::Matrix4d CAM, Eigen::Matrix4d Tco,
        std::vector<zeus_msgs::BoundingBox3D> dets);

    /*!
       \brief This function converts a metric point (x,y,z) in the camera frame into (u,v) in the BEV image in pixels.
    */
    void convert_bev(Eigen::Vector4d xbar, std::vector<double> &data);

    /*!
       \brief Given a metric cuboid defined in the camera frame, this function draws a 2D box onto the BEV visualization
    */
    void draw_box_bev(cv::Mat &bev, std::vector<float> cuboid_parameters, cv::Scalar color);
};

#endif  // OBJECT_VISUALIZER_OBJECT_VISUALIZER_NODE_H
