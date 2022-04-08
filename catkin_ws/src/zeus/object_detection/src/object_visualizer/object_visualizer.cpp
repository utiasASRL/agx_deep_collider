// Author: Keenan Burnett
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <object_visualizer/object_visualizer_node.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>
// #include "utils/json_map_reader.hpp"
// #include "utils/geometry_utils.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_visualizer");
    ros::NodeHandle nh;
    // Get parameters
    std::string map_name, camera_frame, camera_topic, bb_topic, output_append, lidar_topic;
    std::string node_name = ros::this_node::getName();
    // nh.getParam(node_name + "/map_name",        map_name);
    nh.getParam(node_name + "/camera_frame",    camera_frame);
    nh.getParam(node_name + "/camera_topic",    camera_topic);
    nh.getParam(node_name + "/bb_topic",        bb_topic);
    nh.getParam(node_name + "/output_append",   output_append);
    nh.getParam(node_name + "/lidar_topic",     lidar_topic);
    // Read map
    // map::JsonMapReader reader;
    // std::string file_path = ros::package::getPath("map") + "/res/" + std::string(map_name) + ".json";
    // ROS_INFO_STREAM("[OBJ] Loading map from: " << file_path);
    // map::RoadMap road_map = reader.readMap(file_path);
    // Publishers
    ros::Publisher persp_pub = nh.advertise<sensor_msgs::Image>("/Debug/PerspObjects" + output_append, 10);
    ros::Publisher bev_pub = nh.advertise<sensor_msgs::Image>("/Debug/BevObjects" + output_append, 10);
    // Subscribers
    message_filters::Subscriber<sensor_msgs::Image> sub_img(nh, camera_topic, 4);
    message_filters::Subscriber<zeus_msgs::Detections3D> sub_det(nh, bb_topic, 4);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_scan(nh, lidar_topic, 4);
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom(nh, "/navsat/odom", 40);
    // Initialize node object
    ObjectVisualizerNode myNode(persp_pub, bev_pub, nh);
    myNode.set_node_name();
    myNode.get_ros_parameters();
    myNode.initialize_transforms();
    myNode.initialize_roads();
    XmlRpc::XmlRpcValue P, P2, P3, P4;
    nh.getParam(node_name + "/P", P);
    nh.getParam(node_name + "/P2", P2);
    nh.getParam(node_name + "/P3", P3);
    nh.getParam(node_name + "/P4", P4);
    Eigen::Matrix4f CAM;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double temp = P[4 * i + j];
            if (camera_frame == "mono_link2")
                temp = P2[4 * i + j];
            else if (camera_frame == "mono_link3")
                temp = P3[4 * i + j];
            else if (camera_frame == "mono_link4")
                temp = P4[4 * i + j];
            CAM(i, j) = static_cast<float>(temp);
        }
    }
    myNode.set_camera_matrix(CAM);
    // Synchronize subscribers
    typedef message_filters::sync_policies::ApproximateTime<zeus_msgs::Detections3D,
                    sensor_msgs::PointCloud2> sync_policy;
    message_filters::Synchronizer<sync_policy> sync(sync_policy(100), sub_det, sub_scan);
    sync.registerCallback(boost::bind(&ObjectVisualizerNode::callback, myNode, _1, _2));
    ROS_INFO("[OBJ] object visualization running!");
    ros::spin();
    return 0;
}
