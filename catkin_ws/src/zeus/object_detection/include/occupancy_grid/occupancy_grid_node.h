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
#ifndef OCCUPANCY_GRID_OCCUPANCY_GRID_NODE_H
#define OCCUPANCY_GRID_OCCUPANCY_GRID_NODE_H
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <zeus_msgs/BoundingBox3D.h>
#include <zeus_msgs/Detections3D.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>
#include <chrono>  // NOLINT [build/c++11]
#include <string>
#include <vector>
#include "types/zeus_pcl_types.hpp"
#include "utils/transform_utils.hpp"

//* OccupancyGridNode
/**
* \brief This ROS node receives LIDAR pointclouds and outputs 3D object centroids and cuboids based on an occupancy grid
* Objects are obtained through connected-component clustering on the occupancy grid.
* The occupancy grid at each time instance is published for debug purposes.
*
*/
class OccupancyGridNode {
 public:
    OccupancyGridNode(ros::Publisher det_pub, ros::Publisher bev_pub, ros::NodeHandle nh) : det_pub_(det_pub),
        bev_pub_(bev_pub), nh_(nh) {}
    void set_node_name() {node_name = ros::this_node::getName();}

    /*!
       \brief This callback first extracts the ground plane either using RANSAC or the Himmelsbach algo and then
       creates a top-down 2D occupancy grid. Connected components clustering is then used to extract object positions
       and lateral extents.
       \param scan A 3D pointcloud output by a velodyne LIDAR.
    */
    void callback(const sensor_msgs::PointCloud2ConstPtr& scan);

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
    ros::Publisher det_pub_;
    ros::Publisher bev_pub_;
    ros::NodeHandle nh_;
    zeus_tf::tfBufferPtr tfBuffer;
    zeus_tf::tfListenerPtr tfListener;
    std::string node_name;
    Eigen::Matrix4d Tc2_v = Eigen::Matrix4d::Identity();        /*!< Transform from velodyne to c2 */
    Eigen::Matrix4d T_bev_v = Eigen::Matrix4d::Zero();          /*!< Transform from velodyne to bev */
    Eigen::Matrix4d T_v_bev = Eigen::Matrix4d::Zero();          /*!< Transform from bev to velodyne */
    std::vector<double> point_cloud_range = {1.1, 40.0, -14.5, 14.5, -1.8, 3.5};
    std::vector<double> max_object_size = {4.0, 4.0, 4.0};
    std::vector<double> min_object_size = {0.0, 0.0, 0.2};
    float min_object_z = -2.0, max_object_z = 0.5;
    int min_samples = 30;       /*!< Minimum number of points required to keep a cluster */
    int unknown_type = 3;
    // Himmelsbach Ground Plane Extraction Parameters
    float alpha = 5 * M_PI / 180;       /*!< Number of segments = (2 * M_PI) / alpha */
    float tolerance = 0.25;             /*!< Metric distance from the ground plane to be considered a ground point */
    float Tm = 15 * M_PI / 180;         /*!< Slopes greater than this value will be considered non-ground. */
    float Tm_small = 5 * M_PI / 180;    /*!< Slopes less than this value will be checked for being a plateau */
    float Tb = -1.5;                    /*!< Flat regions that are higher than this will be considered non-ground */
    float Trmse = 0.1;                  /*!< If the RSME of the line fit exceeds this value, it will be rejected */
    float Tdprev = 0.25;                /*!< Maximum allowed distance between previous line and start of new line */
    float grid_size = 0.20;             /*!< Discretization used to create the occupancy grid */
    // Occupancy grid variablers
    int H = 250;                        /*!< Height of occupancy grid */
    int W = 150;                        /*!< Width of occupancy grid */
    float unoccupied = 1000;            /*!< Default value of an unoccupied cell */
    int neighbor_size = 8;              /*!< 4-neighbor or 8-neighbor in connected component clustering */
    Eigen::Matrix4f K = Eigen::Matrix4f::Identity();
    Eigen::Vector4f gp_params = Eigen::Vector4f::Zero();
    float max_distance_ransac = 0.25;   /*!< Points closer than this are an inlier to the RANSAC ground plane */
    int max_iterations_ransac = 25;     /*!< Maximum number of iterations to run RANSAC */
    int ground_extract_method = 0;      /*!< 0 --> RANSAC, 1 --> Himmelsbach */

    /*!
       \brief Create the occupancy grids from the pointcloud.
       \param pc The input velodyne pointcloud.
       \param maxz [out] This matrix will be overwitten such that maxz(i, j) = maximum z-value at that grid cell if
            occupied, otherwise it will be set to -unoccupied.
       \param minz [out] This matrix will be overwitten such that minz(i, j) = minimum z-value at that grid cell if
            occupied, otherwise it will be set to unoccupied.
    */
    void initialize_occupancy(const zeus_pcl::PointCloudPtr pc, Eigen::MatrixXf &maxz, Eigen::MatrixXf &minz);

    /*!
       \brief Retrieves the neighbors of a grid cell.
       \param idx The index of the point in the grid cell. idx = row * W + col
       \param neighbors [out] This vector is filled with the indices of the neighbors of idx.
       \pre rosparam neighbor_size determines whether 4-neighbor or 8-neighbor is used.
    */
    void get_neighbors(int idx, std::vector<int> &neighbors);

    /*!
       \brief This function performs connected component clustering over the occupancy grid.
       \param maxz An occupancy grid containing the maximum z-value at each grid cell, -unoccupied otherwise.
       \param clusters [out] This parameter will be filled with a vector of clusters, each cluster consists of a vector
            of indices in the occupancy grid where idx = row * W + col
    */
    void connected_components(const Eigen::MatrixXf &maxz, std::vector<std::vector<int>> &clusters);

    /*!
       \brief For each cluster, this function extracts a 3D centroid and cuboid and creates a ROS message with that list
       \param MAXZ an occupancy grid containing the maximum z-value at each grid cell, -unoccupied otherwise.
       \param MINZ an occupancy grid containing the minimum z-value at each grid cell, unoccupied otherwise.
       \param clusters A vector of clusters, each cluster consists of a vector of indices in the occupancy grid
       \param msg [out] This ROS message will be filled with a vector of 3D objects.
    */
    void extract_boxes(const Eigen::MatrixXf &MAXZ, const Eigen::MatrixXf &MINZ, const std::vector<std::vector<int>> &clusters,
        zeus_msgs::Detections3D &msg);

    /*!
       \brief Creates a debug visualization of the occupancy grid.
    */
    void create_debug_image(const Eigen::MatrixXf &maxz, cv::Mat &img);
};

#endif  // OCCUPANCY_GRID_OCCUPANCY_GRID_NODE_H
