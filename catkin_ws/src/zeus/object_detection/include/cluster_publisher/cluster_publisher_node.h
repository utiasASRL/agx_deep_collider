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
#ifndef CLUSTER_PUBLISHER_CLUSTER_PUBLISHER_NODE_H
#define CLUSTER_PUBLISHER_CLUSTER_PUBLISHER_NODE_H
#include <ros/ros.h>
#include <ros/console.h>
#include <zeus_msgs/BoundingBox2D.h>
#include <zeus_msgs/BoundingBox3D.h>
#include <zeus_msgs/Detections2D.h>
#include <zeus_msgs/Detections3D.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>
#include <chrono>  // NOLINT
#include <string>
#include <vector>
#include "types/zeus_pcl_types.hpp"
#include "utils/transform_utils.hpp"

//* ClusterPublisherNode
/**
* \brief This ROS node receives raw 2D detections (defined wrt an image plane) and the corresponding 3D LIDAR pointcloud
* and outputs a 3D centroid and cuboid for each bounding box in the c2 frame (if a valid one exists).
*
*/
class ClusterPublisherNode {
 public:
    ClusterPublisherNode(ros::Publisher det_pub, ros::NodeHandle nh) : nh_(nh), det_pub_(det_pub) {}
    void set_camera_matrix(Eigen::Matrix4f P_) {P = P_;}
    void set_node_name() {node_name = ros::this_node::getName();}

    /*!
       \brief For each 2D bounding box, this callback attempts to extract a 3D centroid and cuboid (in the c2 frame).
       \param det zeus_msg representing a list of 2D detections (in an image plane).
       \param scan A 3D pointcloud output by a velodyne LIDAR.
       \pre The det and scan topic should be synchronized.
    */
    void callback(const zeus_msgs::Detections2D::ConstPtr& det, const sensor_msgs::PointCloud2ConstPtr& scan);

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

 private:
    std::string node_name = "cluster_publisher";
    ros::NodeHandle nh_;
    ros::Publisher det_pub_;
    zeus_tf::tfBufferPtr tfBuffer;
    zeus_tf::tfListenerPtr tfListener;
    Eigen::Matrix4d Tcv = Eigen::Matrix4d::Identity();          /*!< Transformation from velodyne to camera_frame */
    Eigen::Matrix4d Tc2_c = Eigen::Matrix4d::Identity();        /*!< Transformation from camera_frame to c2 */
    std::vector<double> point_cloud_range = {1.1, 40.0, -14.5, 14.5, -1.8, 3.5};
    float max_distance_ransac = 0.25;       /*!< Points closer than this are an inlier to the RANSAC ground plane */
    int max_iterations_ransac = 25;         /*!< Maximum number of iterations to run RANSAC */
    float cluster_tolerance = 0.25;         /*!< Points closer than this will be clustered together */
    int min_samples = 5;                    /*!< Minimum number of points required to keep a cluster */
    float min_height_ped = 1.0;             /*!< Minimum inferred height of a pedestrian */
    float max_height_ped = 4.0;             /*!< Maximum inferred height of a pedestrian */
    std::vector<double> max_object_size = {4.0, 4.0, 4.0};
    std::vector<double> min_object_size = {0.0, 0.0, 0.2};
    float min_object_z = -2.0, max_object_z = 0.5;
    Eigen::Matrix4f P = Eigen::Matrix4f::Identity();
    std::string camera_frame = "mono_link";
    int camera = 1;
    Eigen::Matrix4f K = Eigen::Matrix4f::Identity();
    Eigen::Vector4f gp_params = Eigen::Vector4f::Zero();

    /*!
       \brief Given a vector of bounding boxes, this function extract a centroid and cuboid for each.
       \param bbs A vector of 2D bounding boxes.
       \param pc A 3D LIDAR pointcloud.
       \param uv A pointcloud where each point has been projected onto the image plane. uv(0) = u, uv(1) = v
       \param centroids [out] This vector will be filled with centroids for the pseudo-3D measurements.
       \param cuboids [out] This vector will be filled with the cuboids for the pseudo-3D measurements.
       \pre The indices of the points in uv and pc should be aligned.
    */
    void cluster_boxes(std::vector<zeus_msgs::BoundingBox2D> bbs, zeus_pcl::PointCloudPtr pc,
       zeus_pcl::PointCloudPtr uv, std::vector<Eigen::Vector4d> &centroids,
       std::vector<std::vector<float>> &cuboids);

    /*!
       \brief Converts the bounding boxes, centroids, and cuboids into a single ros messagwe.
    */
    void convertToRosMessage(std::vector<zeus_msgs::BoundingBox2D> bbs,
       std::vector<Eigen::Vector4d> &centroids, std::vector<std::vector<float>> &cuboids,
       zeus_msgs::Detections3D &outputDetections);
};

#endif  // CLUSTER_PUBLISHER_CLUSTER_PUBLISHER_NODE_H
