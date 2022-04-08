// Author: Keenan Burnett
#include "cluster_publisher/cluster_publisher_node.h"
#include <vector>
#include "utils/zeus_pcl.hpp"

void ClusterPublisherNode::callback(const zeus_msgs::Detections2D::ConstPtr& det,
    const sensor_msgs::PointCloud2ConstPtr& scan) {
    auto start = std::chrono::high_resolution_clock::now();
    zeus_msgs::Detections3D outputDetections;
    // Pre-fill the ROS message
    outputDetections.header.stamp = scan->header.stamp;
    outputDetections.header.frame_id = "/c2";
    outputDetections.camera = camera;
    outputDetections.bbs.clear();
    // If the input detections are empty, publish an empty ROS message and return.
    if (det->bbs.size() == 0) {
        det_pub_.publish(outputDetections);
        return;
    }
    zeus_pcl::PointCloudPtr pc(new zeus_pcl::PointCloud());
    zeus_pcl::PointCloudPtr gp(new zeus_pcl::PointCloud());
    zeus_pcl::PointCloudPtr uv(new zeus_pcl::PointCloud());
    zeus_pcl::fromROSMsg(scan, pc);
    // Restrict points of interest to a rectangular prism defined by point_cloud_range.
    zeus_pcl::passthrough(pc, point_cloud_range[0], point_cloud_range[1], point_cloud_range[2],
        point_cloud_range[3], point_cloud_range[4], point_cloud_range[5]);
    // Remove ground points from pc.
    zeus_pcl::passthrough(pc, gp, 1.0, 40.0, -10, 10.0, -2.75, -1.75);
    zeus_pcl::removeGroundPlane(gp, pc, gp_params, K, max_distance_ransac, max_iterations_ransac);
    // Project pointcloud onto the image plane.
    zeus_pcl::project_points(pc, uv, P * Tcv.cast<float>());  // Transform to camera frame and project onto image plane
    std::vector<zeus_msgs::BoundingBox2D> bbs = det->bbs;
    std::vector<Eigen::Vector4d> centroids;
    std::vector<std::vector<float>> cuboids;
    zeus_pcl::sort_bbs_by_size(bbs);                          // Sort BBs by descending size, helps with overlap
    // Extract a centroid and cuboid for each bounding box.
    cluster_boxes(bbs, pc, uv, centroids, cuboids);
    // Create and publish ROS message
    convertToRosMessage(bbs, centroids, cuboids, outputDetections);
    det_pub_.publish(outputDetections);
    auto stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = stop - start;
    ROS_DEBUG_STREAM("[OBJ] CLUSTER PUBLISHER TIME: " << elapsed.count());
}

void ClusterPublisherNode::get_ros_parameters() {
    nh_.getParam(node_name + "/camera_frame",            camera_frame);
    nh_.getParam(node_name + "/point_cloud_range",       point_cloud_range);
    nh_.getParam(node_name + "/MAX_DISTANCE_RANSAC",     max_distance_ransac);
    nh_.getParam(node_name + "/MAX_ITERATIONS_RANSAC",   max_iterations_ransac);
    nh_.getParam(node_name + "/CLUSTER_TOLERANCE",       cluster_tolerance);
    nh_.getParam(node_name + "/MIN_SAMPLES",             min_samples);
    nh_.getParam(node_name + "/min_height_ped",          min_height_ped);
    nh_.getParam(node_name + "/max_height_ped",          max_height_ped);
    nh_.getParam(node_name + "/max_object_size",         max_object_size);
    nh_.getParam(node_name + "/min_object_size",         min_object_size);
    nh_.getParam(node_name + "/max_object_z",            max_object_z);
    nh_.getParam(node_name + "/min_object_z",            min_object_z);
    float kalman_gain = 0.5;
    nh_.getParam(node_name + "/GROUND_KALMAN_GAIN",      kalman_gain);
    camera = 1;
    if (camera_frame == "mono_link2")
        camera = 2;
    if (camera_frame == "mono_link3")
        camera = 3;
    if (camera_frame == "mono_link4")
        camera = 4;
    K = Eigen::Matrix4f::Identity() * kalman_gain;
    gp_params.matrix() << 0.0, 0.0, -1.0, -1.70;
}

void ClusterPublisherNode::initialize_transforms() {
    tfBuffer = zeus_tf::tfBufferPtr(new zeus_tf::tfBuffer());
    tfListener = zeus_tf::tfListenerPtr(new zeus_tf::tfListener(*tfBuffer));
    zeus_tf::get_transform(tfBuffer, camera_frame, "c2", Tc2_c);
    zeus_tf::get_transform(tfBuffer, "velodyne", camera_frame, Tcv);
}

static void convert_indices(std::vector<int> parent_indices, std::vector<int> &child_indices) {
    for (uint i = 0; i < child_indices.size(); i++) {
        child_indices[i] = parent_indices[child_indices[i]];
    }
}

void ClusterPublisherNode::cluster_boxes(std::vector<zeus_msgs::BoundingBox2D> bbs, zeus_pcl::PointCloudPtr pc,
    zeus_pcl::PointCloudPtr uv, std::vector<Eigen::Vector4d> &centroids, std::vector<std::vector<float>> &cuboids) {
    std::vector<int> clustered_so_far;
    for (int i = 0; i < (int)bbs.size(); i++) {
        zeus_pcl::PointCloudPtr bb_pts (new zeus_pcl::PointCloud());
        zeus_pcl::PointCloudPtr uv_bb (new zeus_pcl::PointCloud());
        std::vector<int> indices;
        zeus_pcl::passthrough(uv, uv_bb, bbs[i].cx - bbs[i].w/2, bbs[i].cx + bbs[i].w/2,
            bbs[i].cy - bbs[i].h/2, bbs[i].cy + bbs[i].h/2, indices);
        zeus_pcl::get_subset_unclustered(indices, clustered_so_far);
        zeus_pcl::extract_points(pc, bb_pts, indices);
        if ((int)bb_pts->size() > 0) {
            std::vector<std::vector<int>> clusters;
            zeus_pcl::cluster_point_cloud(bb_pts, min_samples, cluster_tolerance, clusters);
            if (clusters.size() > 0) {
                Eigen::Vector4d x;
                int idx = zeus_pcl::filter_clusters(clusters, bb_pts, bbs[i], P, x, min_height_ped, max_height_ped);
                if (idx >= 0) {
                    zeus_pcl::PointCloudPtr bb_pts_cluster (new zeus_pcl::PointCloud());
                    zeus_pcl::extract_points(bb_pts, bb_pts_cluster, clusters[idx]);
                    std::vector<float> cuboid;
                    centroids.push_back(zeus_pcl::get_centroid(bb_pts_cluster, cuboid));
                    cuboids.push_back(cuboid);
                    convert_indices(indices, clusters[idx]);
                    clustered_so_far.insert(clustered_so_far.end(), clusters[idx].begin(), clusters[idx].end());
                    continue;
                }
            }
        }
        Eigen::Vector4d centroid;
        centroid << -1, -1, -1, -1;
        centroids.push_back(centroid);
        cuboids.push_back(std::vector<float>(1, -1.0));
    }
}

void ClusterPublisherNode::convertToRosMessage(std::vector<zeus_msgs::BoundingBox2D> bbs,
    std::vector<Eigen::Vector4d> &centroids, std::vector<std::vector<float>> &cuboids,
    zeus_msgs::Detections3D &outputDetections) {
    for (int i = 0; i < (int)bbs.size(); i++) {
        Eigen::Vector4d xbar = centroids[i];
        if (xbar(3, 0) < 0)  // A LIDAR cluster was not found
            continue;
        xbar = Tcv * xbar;
        xbar(1, 0) = (bbs[i].cy - P(1, 2)) * fabs(xbar(2, 0)) / P(1, 1);
        xbar = Tc2_c * xbar;
        if (xbar(2, 0) < min_object_z || xbar(2, 0) > max_object_z)
            continue;
        zeus_msgs::BoundingBox3D detection;
        detection.x = xbar(0, 0);
        detection.y = xbar(1, 0);
        detection.z = xbar(2, 0);
        detection.w = fabs(xbar(0, 0)) * (float)bbs[i].w / P(0, 0);
        detection.h = fabs(xbar(0, 0)) * (float)bbs[i].h / P(1, 1);
        std::vector<float> cuboid = cuboids[i];
        detection.l = fabs(cuboid[3] - cuboid[0]);
        detection.type = bbs[i].type;
        detection.confidence = bbs[i].confidence;
        detection.camera = camera;
        outputDetections.bbs.push_back(detection);
    }
}
