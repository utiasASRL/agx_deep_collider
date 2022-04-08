// Author: Keenan Burnett
#include "secondary_clustering/secondary_clustering_node.h"
#include <vector>
#include "utils/zeus_pcl.hpp"

void SecondaryClusteringNode::callback(const sensor_msgs::PointCloud2ConstPtr& scan) {
    auto start = std::chrono::high_resolution_clock::now();
    zeus_pcl::PointCloudPtr pc(new zeus_pcl::PointCloud());
    zeus_pcl::PointCloudPtr gp(new zeus_pcl::PointCloud());
    zeus_pcl::fromROSMsg(scan, pc);
    zeus_pcl::passthrough(pc, point_cloud_range[0], point_cloud_range[1], point_cloud_range[2],
        point_cloud_range[3], point_cloud_range[4], point_cloud_range[5]);
    zeus_pcl::passthrough(pc, gp, -5, 5, -5, 5, -0.8, -0.4);
    zeus_pcl::removeGroundPlane(gp, pc, gp_params, K, max_distance_ransac, max_iterations_ransac);
    zeus_pcl::randomDownSample(pc, 0.5);
    // Create ROS message
    zeus_msgs::Detections3D outputDetections;
    outputDetections.header.stamp = scan->header.stamp;
    // outputDetections.header.frame_id = "/c2";
    outputDetections.header.frame_id = "/velodyne";
    outputDetections.camera = 1;
    outputDetections.bbs.clear();
    // Secondary Clustering
    secondary_clustering(pc, outputDetections);
    // Publish ROS Message
    det_pub_.publish(outputDetections);
    auto stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = stop - start;
    ROS_INFO_STREAM("[OBJ] SECONDARY CLUSTERING TIME: " << elapsed.count());
}

void SecondaryClusteringNode::get_ros_parameters() {
    nh_.getParam(node_name + "/secondary_cloud_range",   point_cloud_range);
    nh_.getParam(node_name + "/MAX_DISTANCE_RANSAC",     max_distance_ransac);
    nh_.getParam(node_name + "/MAX_ITERATIONS_RANSAC",   max_iterations_ransac);
    nh_.getParam(node_name + "/CLUSTER_TOLERANCE_SECONDARY", cluster_tolerance);
    nh_.getParam(node_name + "/MIN_SAMPLES_SECONDARY",   min_samples);
    nh_.getParam(node_name + "/max_object_size",         max_object_size);
    nh_.getParam(node_name + "/min_object_size",         min_object_size);
    nh_.getParam(node_name + "/max_object_z",            max_object_z);
    nh_.getParam(node_name + "/min_object_z",            min_object_z);
    nh_.getParam(node_name + "/unknown_type",            unknown_type);
    float kalman_gain = 0.5;
    nh_.getParam(node_name + "/GROUND_KALMAN_GAIN",      kalman_gain);
    K = Eigen::Matrix4f::Identity() * kalman_gain;
    gp_params.matrix() << 0.0, 0.0, -1, -0.65;
}

// void SecondaryClusteringNode::initialize_transforms() {
//     tfBuffer = zeus_tf::tfBufferPtr(new zeus_tf::tfBuffer());
//     tfListener = zeus_tf::tfListenerPtr(new zeus_tf::tfListener(*tfBuffer));
//     zeus_tf::get_transform(tfBuffer, "velodyne", "c2", Tc2_v);
// }

void SecondaryClusteringNode::secondary_clustering(zeus_pcl::PointCloudPtr pc,
    zeus_msgs::Detections3D &outputDetections) {
    std::vector<std::vector<int>> clusters;
    if (pc->size() > 0)
        zeus_pcl::cluster_point_cloud(pc, min_samples, cluster_tolerance, clusters);
    for (uint i = 0; i < clusters.size(); i++) {
        zeus_pcl::PointCloudPtr cluster_pts (new zeus_pcl::PointCloud());
        zeus_pcl::extract_points(pc, cluster_pts, clusters[i]);
        std::vector<float> cuboid;
        Eigen::Vector4d centroid =  zeus_pcl::get_centroid(cluster_pts, cuboid);
        double w, l, h;
        // cuboid: {min_x, min_y, min_z, max_x, max_y, max_z}
        l = fabs(cuboid[3] - cuboid[0]);
        w = fabs(cuboid[4] - cuboid[1]);
        h = fabs(cuboid[5] - cuboid[2]);
        if (l > max_object_size[0] || w > max_object_size[1] || h > max_object_size[2] ||
            l < min_object_size[0] || w < min_object_size[1] || h < min_object_size[2])
            continue;
        if (centroid(2, 0) < min_object_z || centroid(2, 0) > max_object_z)
            continue;
        zeus_msgs::BoundingBox3D detection;
        // centroid = Tc2_v * centroid;
        detection.x = centroid(0, 0);
        detection.y = centroid(1, 0);
        detection.z = centroid(2, 0);
        detection.l = l;
        detection.w = w;
        detection.h = h;
        detection.type = unknown_type;
        detection.confidence = 0.9;
        detection.camera = 1;
        outputDetections.bbs.push_back(detection);
    }
}
