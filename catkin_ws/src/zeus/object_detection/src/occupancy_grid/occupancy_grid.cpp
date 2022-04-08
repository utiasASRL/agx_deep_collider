// Author: Keenan Burnett
#include <occupancy_grid/occupancy_grid_node.h>
#include <ros/console.h>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "occupancy_grid");
    ros::NodeHandle nh;
    // Get parameters
    std::string output_append, lidar_topic;
    std::string node_name = ros::this_node::getName();
    nh.getParam(node_name + "/lidar_topic",     lidar_topic);
    nh.getParam(node_name + "/output_append",   output_append);
    // Publishers
    ros::Publisher det_pub = nh.advertise<zeus_msgs::Detections3D>("/Object/RawDetections3D" + output_append, 10);
    ros::Publisher bev_pub = nh.advertise<sensor_msgs::Image>("/Debug/Occupancy", 10);
    // Initialize cluster publisher node object
    OccupancyGridNode myNode(det_pub, bev_pub, nh);
    myNode.set_node_name();
    myNode.get_ros_parameters();
    myNode.initialize_transforms();
    // Subscribers
    ros::Subscriber sub = nh.subscribe(lidar_topic, 2, &OccupancyGridNode::callback, &myNode);
    ROS_INFO("[OBJ] occupancy_grid running!");
    ros::spin();
    return 0;
}
