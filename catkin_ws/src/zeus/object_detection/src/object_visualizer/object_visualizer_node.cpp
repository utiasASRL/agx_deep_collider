// Author: Keenan Burnett
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "object_visualizer/object_visualizer_node.h"
#include <opencv2/core.hpp>
#include "utils/zeus_pcl.hpp"
// #include "utils/geometry_utils.hpp"
// #include "utils/map_vis.hpp"
#include "utils/zeus_vis.hpp"

void ObjectVisualizerNode::callback(const zeus_msgs::Detections3D::ConstPtr& det,
    const sensor_msgs::PointCloud2ConstPtr& scan) {
    auto start = std::chrono::high_resolution_clock::now();
    zeus_tf::get_transform(tfBuffer, "map", "velodyne", Tvm);
    // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    // cv::Mat persp_img = cv_ptr->image;
    zeus_pcl::PointCloudPtr pc(new zeus_pcl::PointCloud());
    zeus_pcl::PointCloudPtr gp(new zeus_pcl::PointCloud());
    zeus_pcl::PointCloudPtr uv(new zeus_pcl::PointCloud());
    zeus_pcl::fromROSMsg(scan, pc);
    // Restrict points of interest to a rectangular prism defined by point_cloud_range.
    zeus_pcl::passthrough(pc, point_cloud_range[0], point_cloud_range[1], point_cloud_range[2],
        point_cloud_range[3], point_cloud_range[4], point_cloud_range[5]);
    // Remove ground points from pc.
    zeus_pcl::passthrough(pc, gp, -5, 5, -5, 5, -0.8, -0.4);
    zeus_pcl::removeGroundPlane(gp, pc, gp_params, K, max_distance_ransac, max_iterations_ransac);
    // Transform to camera frame and project onto image plane
    // zeus_pcl::project_points(pc, uv, P * Tcv.cast<float>());
    // Project pointcloud onto the image plane.
    // if (show_points)
    //     zeus_pcl::project_onto_image(persp_img, uv);
    // Resize the visualization to use less compute
    // int image_height = persp_img.size().height;
    // int image_width = persp_img.size().width;
    // cv::resize(persp_img, persp_img, cv::Size(int(image_width / viz_divide), int(image_height / viz_divide)));
    std::vector<zeus_msgs::BoundingBox3D> dets = det->bbs;
    // Create BEV image
    // Eigen::Matrix4d Toi = Eigen::Matrix4d::Identity();
    // zeus_tf::get_odom_tf(*odom, Toi);
    // Eigen::Matrix4d Tco = zeus_tf::get_inverse_tf(Toi * Tic2);
    cv::Mat bev_img;
    zeus_pcl::create_bev(pc, h_metric, w_metric, meter_to_pixel, bev_img, T_bev_v);
    // zeus_pcl::draw_hfov(bev_img, image_width, P(0, 0), cam_center);
    // Draw the road map on the BEV image
    // if (show_map) {
    //     auto av_pose = map::Pose2(odom->pose.pose, M_PI_2);
    //     road_id = map::find_current_road_segment(road_map_, av_pose, false, road_id, road_polygons, roads);
    //     map::draw_road(bev_img, road_polygons, roads, w_metric, h_metric,
    //         meter_to_pixel, odom->pose.pose.position.z, T_bev_v * Tvi * zeus_tf::get_inverse_tf(Toi), road_id);
    // }
    // Draw the detections on the perspective and BEV images
    show_tracked_dets(bev_img, P.cast<double>(), Tvm, dets);
    cv_bridge::CvImage out_msg;
    out_msg.encoding = "bgr8";
    // out_msg.image = persp_img;
    out_msg.header.stamp = det->header.stamp;
    // persp_pub_.publish(out_msg.toImageMsg());
    out_msg.image = bev_img;
    bev_pub_.publish(out_msg.toImageMsg());
    auto stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = stop - start;
    ROS_DEBUG_STREAM("[OBJ] OBJECT VIS TIME: " << elapsed.count());
}

void ObjectVisualizerNode::get_ros_parameters() {
    nh_.getParam(node_name + "/camera_frame",            camera_frame);
    nh_.getParam(node_name + "/MAX_DISTANCE_RANSAC",     max_distance_ransac);
    nh_.getParam(node_name + "/MAX_ITERATIONS_RANSAC",   max_iterations_ransac);
    nh_.getParam(node_name + "/viz_divide",              viz_divide);
    nh_.getParam(node_name + "/SHOW_RAW_DETS",           show_raw_dets);
    nh_.getParam(node_name + "/SHOW_POINTS",             show_points);
    nh_.getParam(node_name + "/SHOW_CUBES",              show_cubes);
    nh_.getParam(node_name + "/SHOW_MAP",                show_map);
    nh_.getParam(node_name + "/meter_to_pixel",          meter_to_pixel);
    nh_.getParam(node_name + "/SHOW_DETAILED_TEXT",      show_detailed_text);
    nh_.getParam(node_name + "/point_cloud_range",       point_cloud_range);
    nh_.getParam(node_name + "/unknown_type",            unknown_type);
    float kalman_gain = 0.5;
    nh_.getParam(node_name + "/GROUND_KALMAN_GAIN",      kalman_gain);
    K = Eigen::Matrix4f::Identity() * kalman_gain;
    gp_params.matrix() << 0.0, 0.0, -1.0, -0.65;
}

void ObjectVisualizerNode::initialize_transforms() {
    tfBuffer = zeus_tf::tfBufferPtr(new zeus_tf::tfBuffer());
    tfListener = zeus_tf::tfListenerPtr(new zeus_tf::tfListener(*tfBuffer));
    // zeus_tf::get_transform(tfBuffer, "c2", "imu_link", Tic2);
    // zeus_tf::get_transform(tfBuffer, "imu_link", "velodyne", Tvi);
    // zeus_tf::get_transform(tfBuffer, "c2", camera_frame, Tcc2);
    // zeus_tf::get_transform(tfBuffer, "velodyne", camera_frame, Tcv);
    // Tvc = zeus_tf::get_inverse_tf(Tcv);
    Eigen::Matrix4d Temp;
    // zeus_tf::get_transform(tfBuffer, "mono_link", "mono_link3", Temp);
    // float c_theta = Temp(0, 0);
    // float s_theta = Temp(2, 0);
    // R3_c2 << c_theta, s_theta, 0, -s_theta, c_theta, 0, 0, 0, 1;
    // zeus_tf::get_transform(tfBuffer, "mono_link", "mono_link4", Temp);
    // c_theta = Temp(0, 0);
    // s_theta = Temp(2, 0);
    // R4_c2 << c_theta, s_theta, 0, -s_theta, c_theta, 0, 0, 0, 1;
    T_bev_v(0, 1) = -1;
    T_bev_v(1, 0) = -1;
    T_bev_v(2, 2) = -1;
    T_bev_v(3, 3) = 1;
    T_bev_v(0, 3) = point_cloud_range[3];
    T_bev_v(1, 3) = point_cloud_range[1];
    h_metric = fabs(point_cloud_range[1] - point_cloud_range[0]);
    w_metric = fabs(point_cloud_range[3] - point_cloud_range[2]);
    Eigen::Vector4d cc = {0, 0, 0, 1};
    cc = T_bev_v * Tvc * cc;
    cam_center = cv::Point(int(meter_to_pixel * cc(0, 0)), int(meter_to_pixel * cc(1, 0)));
}

void ObjectVisualizerNode::initialize_roads() {
    // road_polygons = extract_polygons_from_roadmap(road_map_);
    // roads = road_map_.getRoads();
}

void ObjectVisualizerNode::convert_bev(Eigen::Vector4d xbar, std::vector<double> &data) {
    xbar = T_bev_v * Tvc * xbar;
    data[0] = meter_to_pixel * xbar(0, 0);
    data[1] = meter_to_pixel * xbar(1, 0);
}

// x, y, z in camera frame
void ObjectVisualizerNode::draw_box_bev(cv::Mat &bev, std::vector<float> cuboid_parameters, cv::Scalar color) {
    float min_x, max_x, min_z, max_z;
    float x, z, w, l;
    x = cuboid_parameters[0];
    z = cuboid_parameters[2];
    w = cuboid_parameters[3];
    l = cuboid_parameters[4];
    min_x = x - w / 2;
    max_x = x + w / 2;
    min_z = z - l / 2;
    max_z = z + l / 2;
    Eigen::Vector4d xbar;
    xbar << min_x, 0.0, max_z, 1.0;
    std::vector<double> p1(2, 0.0);
    convert_bev(xbar, p1);
    xbar << max_x, 0.0, min_z, 1.0;
    std::vector<double> p2(2, 0.0);
    convert_bev(xbar, p2);
    cv::rectangle(bev, cv::Point(int(p1[0]), int(p1[1])), cv::Point(int(p2[0]), int(p2[1])), color, 2);
}

// Draws a velocity vector at center (cx,cy) with length (vx,vy)
void quiver(cv::Mat img, int cx, int cy, double vx, double vy, double scale) {
    cv::Point p1(cx, cy);
    cv::Point p2(int(cx + vx * scale), int(cy + vy * scale));
    cv::arrowedLine(img, p1, p2, cv::Scalar(255, 0, 0), 2);
}

void ObjectVisualizerNode::show_tracked_dets(cv::Mat &bev, Eigen::Matrix4d CAM,
    Eigen::Matrix4d Tvm, std::vector<zeus_msgs::BoundingBox3D> dets) {

    for (int i = 0; i < (int)dets.size(); i++) {
        cv::Scalar color(0, 255, 0);            // Green
        if ((dets[i].type == unknown_type && dets[i].confidence < 0.5) ||
            (dets[i].type == 1 && dets[i].confidence < 0.25)) {
            color = cv::Scalar(0, 165, 255);    // Orange
        }
        // int type = dets[i].type;
        Eigen::Vector4d xbar;
        xbar << dets[i].x, dets[i].y, dets[i].z, 1.0;
        Eigen::MatrixXd vbar = Eigen::MatrixXd::Zero(3, 1);
        vbar(0, 0) = dets[i].x_dot;
        vbar(1, 0) = dets[i].y_dot;
        vbar(2, 0) = 1.0;
        xbar = Tvm * xbar;
        Eigen::MatrixXd Rvm = Tvm.block(0, 0, 3, 3);
        vbar = Rvm * vbar;
        // if (camera_frame == "mono_link3")
        //     vbar = R3_c2 * vbar;
        // if (camera_frame == "mono_link4")
        //     vbar = R4_c2 * vbar;
        // TODO(keenan_burnett): vbar = Rco * (vbar - vi_vi) - omegacross*xbar(1:3);
        // std::stringstream stringStream3;
        // stringStream3 << dets[i].confidence;
        // std::string text3 = stringStream3.str();
        std::vector<double> data(2, 0.0);
        xbar = Tcc2 * xbar;     // convert from c2 to camera frame
        convert_bev(xbar, data);
        int h = 0, w = 0;
        // Draw stuff on persp image
        // w = CAM(0, 0) * dets[i].w / xbar(2, 0);
        // h = CAM(1, 1) * dets[i].h / xbar(2, 0);
        // w = int(w / viz_divide);
        // h = int(h / viz_divide);
        // int cx = 0, cy = 0;
        // Eigen::MatrixXd uv = CAM * xbar;
        // uv = uv / uv(2, 0);
        // cx = int(uv(0, 0) / viz_divide);
        // cy = int(uv(1, 0) / viz_divide);
        // std::vector<int> box = {cx, cy, w, h};
        std::vector<float> cuboid_parameters = {(float)xbar(0, 0), (float)xbar(1, 0), (float)xbar(2, 0),
                                            dets[i].w, dets[i].l, dets[i].h};
        // if (cx > 0 && cy > 0 && cx < persp.size().width && cy < persp.size().height) {
        //     if (show_cubes) {
        //         zeus_pcl::draw_cube(persp, cuboid_parameters, P, viz_divide, color);
        //     } else {
        //         cv::rectangle(persp, cv::Point(int(box[0] - box[2]/2),
        //             int(box[1] - box[3]/2)), cv::Point(int(box[0] + box[2]/2),
        //             int(box[1] + box[3]/2)), color, 2);
        //     }
        //     if (type != unknown_type) {
        //         cv::putText(persp, type_names[type], cv::Point(box[0], box[1] + box[3]/2 + 25),
        //             cv::FONT_HERSHEY_PLAIN, 1.5, color, 2.0);
        //         if (show_detailed_text) {
        //             cv::putText(persp, text3, cv::Point(box[0], box[1] + box[3]/2 + 45),
        //                 cv::FONT_HERSHEY_PLAIN, 1.5, color, 2.0);
        //         }
        //     }
        // }
        // Draw stuff on BEV image
        int u = int(data[0]);
        int v = int(data[1]);
        w = int(w_metric * meter_to_pixel);
        h = int(h_metric * meter_to_pixel);
        if (u > 0 && v > 0 && u < w && v < h) {
            cv::drawMarker(bev, cv::Point(u, v), color, 3, 5, 2);
            draw_box_bev(bev, cuboid_parameters, color);
            // if (type != unknown_type)
            //     cv::putText(bev, type_names[type], cv::Point(u, v + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            quiver(bev, u, v, -vbar(1, 0), -vbar(0, 0), 20);
        }
    }
}
