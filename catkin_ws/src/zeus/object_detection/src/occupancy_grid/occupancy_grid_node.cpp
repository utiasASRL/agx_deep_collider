// Author: Keenan Burnett
#include "occupancy_grid/occupancy_grid_node.h"
#include <vector>
#include <deque>
#include "utils/zeus_pcl.hpp"
#include <opencv2/core/eigen.hpp>

void OccupancyGridNode::callback(const sensor_msgs::PointCloud2ConstPtr& scan) {
    auto start = std::chrono::high_resolution_clock::now();
    zeus_pcl::PointCloudPtr pc(new zeus_pcl::PointCloud());
    zeus_pcl::fromROSMsg(scan, pc);
    zeus_pcl::passthrough(pc, point_cloud_range[0], point_cloud_range[1], point_cloud_range[2],
                          point_cloud_range[3], point_cloud_range[4], point_cloud_range[5]);
    if (ground_extract_method == 0) {
        // RANSAC-Based Ground Plane Extraction
        zeus_pcl::PointCloudPtr gp(new zeus_pcl::PointCloud());
        zeus_pcl::passthrough(pc, gp, -5, 5, -5, 5, -0.8, -0.4);
        zeus_pcl::removeGroundPlane(gp, pc, gp_params, K, max_distance_ransac, max_iterations_ransac);
    } else if (ground_extract_method == 1) {
        // Himmelsbach algorithm for ground plane extraction
        zeus_pcl::removeGroundPlane2(pc, alpha, tolerance, Tm, Tm_small, Tb, Trmse, Tdprev);
    }
    zeus_pcl::randomDownSample(pc, 0.5);
    // Create 2D occupancy grids
    Eigen::MatrixXf maxz, minz;
    initialize_occupancy(pc, maxz, minz);
    // Connected component clustering
    std::vector<std::vector<int>> clusters;
    connected_components(maxz, clusters);
    // Extract 3D Bounding Boxes from connected components
    zeus_msgs::Detections3D outputDetections;
    extract_boxes(maxz, minz, clusters, outputDetections);
    // Publish ROS Message
    outputDetections.header.stamp = scan->header.stamp;
    outputDetections.header.frame_id = "/velodyne";
    outputDetections.camera = 1;
    det_pub_.publish(outputDetections);
    cv::Mat img;
    create_debug_image(maxz, img);
    cv_bridge::CvImage out_msg;
    out_msg.encoding = "mono8";
    out_msg.image = img;
    out_msg.header.stamp = scan->header.stamp;
    bev_pub_.publish(out_msg.toImageMsg());
    auto stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = stop - start;
    ROS_DEBUG_STREAM("[OBJ] OCCUPANCY GRID CLUSTERING TIME: " << elapsed.count());
}

void OccupancyGridNode::get_ros_parameters() {
    nh_.getParam(node_name + "/point_cloud_range",       point_cloud_range);
    nh_.getParam(node_name + "/MIN_SAMPLES_SECONDARY",   min_samples);
    nh_.getParam(node_name + "/max_object_size",         max_object_size);
    nh_.getParam(node_name + "/min_object_size",         min_object_size);
    nh_.getParam(node_name + "/max_object_z",            max_object_z);
    nh_.getParam(node_name + "/min_object_z",            min_object_z);
    nh_.getParam(node_name + "/unknown_type",            unknown_type);
    nh_.getParam(node_name + "/alpha",                   alpha);
    nh_.getParam(node_name + "/tolerance",               tolerance);
    nh_.getParam(node_name + "/Tm",                      Tm);
    nh_.getParam(node_name + "/Tm_small",                Tm_small);
    nh_.getParam(node_name + "/Tb",                      Tb);
    nh_.getParam(node_name + "/Trmse",                   Trmse);
    nh_.getParam(node_name + "/Tdprev",                  Tdprev);
    nh_.getParam(node_name + "/neighbor_size",           neighbor_size);
    nh_.getParam(node_name + "/grid_size",               grid_size);
    nh_.getParam(node_name + "/MAX_DISTANCE_RANSAC",     max_distance_ransac);
    nh_.getParam(node_name + "/MAX_ITERATIONS_RANSAC",   max_iterations_ransac);
    nh_.getParam(node_name + "/ground_extract_method",   ground_extract_method);
    float kalman_gain = 0.5;
    nh_.getParam(node_name + "/GROUND_KALMAN_GAIN",      kalman_gain);
    K = Eigen::Matrix4f::Identity() * kalman_gain;
    gp_params.matrix() << 0.0, 0.0, -1, -0.65;
}

void OccupancyGridNode::initialize_occupancy(const zeus_pcl::PointCloudPtr pc, Eigen::MatrixXf &maxz, Eigen::MatrixXf &minz) {
    maxz = Eigen::MatrixXf::Constant(H, W, -unoccupied);
    minz = Eigen::MatrixXf::Constant(H, W, unoccupied);
    for (uint i = 0; i < pc->size(); i++) {
        Eigen::Vector4d xbar = {pc->points[i].x, pc->points[i].y, pc->points[i].z, 1.0};
        xbar = T_bev_v * xbar;
        const int col = xbar(0, 0) / grid_size;
        const int row = xbar(1, 0) / grid_size;
        if (col < 0 || col >= W || row < 0 || row >= H)
            continue;
        if (pc->points[i].z < minz(row, col))
            minz(row, col) = pc->points[i].z;
        if (pc->points[i].z > maxz(row, col))
            maxz(row, col) = pc->points[i].z;
    }
}

void OccupancyGridNode::create_debug_image(const Eigen::MatrixXf &maxz, cv::Mat &img) {
    img = cv::Mat::zeros(H, W, CV_8UC1);
    for (uint i = 0; i < maxz.rows(); i++) {
        for (uint j = 0; j < maxz.cols(); j++) {
                if (maxz(i, j) != -unoccupied)
                        img.at<uint8_t>(i, j) = 255;
        }
    }
}

void OccupancyGridNode::get_neighbors(int idx, std::vector<int> &neighbors) {
    const int col = idx % W;
    const int row = floor(idx / W);
    if ((col - 1) >= 0)
        neighbors.push_back(row * W + col - 1);
    if ((row - 1) >= 0)
        neighbors.push_back((row - 1) * W + col);
    if ((row + 1) < H)
        neighbors.push_back((row + 1) * W + col);
    if ((col + 1) < W)
        neighbors.push_back(row * W + col + 1);
    if (neighbor_size > 4) {
        if ((row - 1) >= 0 && (col - 1) >= 0)
            neighbors.push_back((row - 1) * W + col - 1);
        if ((row + 1) < H && (col - 1) >= 0)
            neighbors.push_back((row + 1) * W + col - 1);
        if ((row - 1) >= 0 && (col + 1) < W)
            neighbors.push_back((row - 1) * W + col + 1);
        if ((row + 1) < H && (col + 1) < W)
            neighbors.push_back((row + 1) * W + col + 1);
    }
}

void OccupancyGridNode::connected_components(const Eigen::MatrixXf &maxz, std::vector<std::vector<int>> &clusters) {
    Eigen::MatrixXf traversed = Eigen::MatrixXf::Zero(H, W);
    for (uint i = 0; i < maxz.rows(); i++) {
        for (uint j = 0; j < maxz.cols(); j++) {
            if (maxz(i, j) > -unoccupied && !traversed(i, j)) {
                std::vector<int> cluster;
                std::deque<int> q;
                q.push_back(i * W + j);
                // Breadth-First Search for neighbors (8-neighbors)
                while (q.size() > 0) {
                    int idx = q[0];
                    q.pop_front();
                    cluster.push_back(idx);
                    std::vector<int> neighbors;
                    get_neighbors(idx, neighbors);
                    for (int neighbor : neighbors) {
                        int col = neighbor % W;
                        int row = floor(neighbor / W);
                        if (!traversed(row, col) && maxz(row, col) > -unoccupied) {
                            traversed(row, col) = 1;
                            q.push_back(neighbor);
                        }
                    }
                }
                clusters.push_back(cluster);
            }
            traversed(i, j) = 1;
        }
    }
}

void OccupancyGridNode::extract_boxes(const Eigen::MatrixXf &MAXZ, const Eigen::MatrixXf &MINZ,
    const std::vector<std::vector<int>> &clusters, zeus_msgs::Detections3D &msg) {
    for (std::vector<int> cluster : clusters) {
        float minx = 1000, miny = 1000, minz = 1000;
        float maxx = -1000, maxy = -1000, maxz = -1000;
        for (int index : cluster) {
            const int col = index % W;
            const int row = floor(index / W);
            if (MINZ(row, col) < minz)
                minz = MINZ(row, col);
            if (MAXZ(row, col) > maxz)
                maxz = MAXZ(row, col);
            Eigen::Vector4d xbar = {col * grid_size, row * grid_size, 0.0, 1.0};
            xbar = T_v_bev * xbar;
            if (xbar(0, 0) < minx)
                minx = xbar(0, 0);
            if (xbar(0, 0) > maxx)
                maxx = xbar(0, 0);
            if (xbar(1, 0) < miny)
                miny = xbar(1, 0);
            if (xbar(1, 0) > maxy)
                maxy = xbar(1, 0);
        }
        const float l = fabs(maxx - minx);
        const float w = fabs(maxy - miny);
        const float h = fabs(maxz - minz);
        float cx = (maxx + minx) / 2;
        float cy = (maxy + miny) / 2;
        float cz = (maxz + minz) / 2;
        if (l > max_object_size[0] || w > max_object_size[1] || h > max_object_size[2] ||
            l < min_object_size[0] || w < min_object_size[1] || h < min_object_size[2])
            continue;
        if (cz < min_object_z || cz > max_object_z)
            continue;
        Eigen::Vector4d xbar = {cx, cy, cz, 1.0};
        // xbar = Tc2_v * xbar;
        zeus_msgs::BoundingBox3D detection;
        detection.x = xbar(0, 0);
        detection.y = xbar(1, 0);
        detection.z = xbar(2, 0);
        detection.l = l;
        detection.w = w;
        detection.h = h;
        detection.type = unknown_type;
        detection.confidence = 0.9;
        detection.camera = 1;
        msg.bbs.push_back(detection);
    }
}

void OccupancyGridNode::initialize_transforms() {
    tfBuffer = zeus_tf::tfBufferPtr(new zeus_tf::tfBuffer());
    tfListener = zeus_tf::tfListenerPtr(new zeus_tf::tfListener(*tfBuffer));
    // zeus_tf::get_transform(tfBuffer, "velodyne", "c2", Tc2_v);
    T_bev_v(0, 1) = -1;
    T_bev_v(1, 0) = -1;
    T_bev_v(2, 2) = -1;
    T_bev_v(3, 3) = 1;
    T_bev_v(0, 3) = point_cloud_range[3];
    T_bev_v(1, 3) = point_cloud_range[1];
    T_v_bev = zeus_tf::get_inverse_tf(T_bev_v);
    H = ceil(fabs(point_cloud_range[1] - point_cloud_range[0]) / grid_size);
    W = ceil(fabs(point_cloud_range[3] - point_cloud_range[2]) / grid_size);
}
