// Author: Keenan Burnett
#include <ros/ros.h>
#include <ros/console.h>
#include <cluster_publisher/cluster_publisher_node.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "cluster_publisher");
    ros::NodeHandle nh;
    // Get parameters
    std::string camera_frame, bb_topic, output_append, lidar_topic;
    std::string node_name = ros::this_node::getName();
    nh.getParam(node_name + "/camera_frame",    camera_frame);
    nh.getParam(node_name + "/bb_topic",        bb_topic);
    nh.getParam(node_name + "/lidar_topic",     lidar_topic);
    nh.getParam(node_name + "/output_append",   output_append);
    // Publishers
    ros::Publisher det_pub = nh.advertise<zeus_msgs::Detections3D>("/Object/RawDetections3D" + output_append, 10);
    // Subscribers
    message_filters::Subscriber<zeus_msgs::Detections2D> sub_det(nh, bb_topic, 2);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_scan(nh, lidar_topic, 2);
    // Initialize cluster publisher node object
    ClusterPublisherNode myNode(det_pub, nh);
    myNode.set_node_name();
    myNode.get_ros_parameters();
    myNode.initialize_transforms();
    // Load in more parameters
    XmlRpc::XmlRpcValue P, P2, P3, P4;
    Eigen::Matrix4f CAM;
    nh.getParam(node_name + "/P", P);
    nh.getParam(node_name + "/P2", P2);
    nh.getParam(node_name + "/P3", P3);
    nh.getParam(node_name + "/P4", P4);
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
    typedef message_filters::sync_policies::ApproximateTime<zeus_msgs::Detections2D,
                    sensor_msgs::PointCloud2> sync_policy;
    message_filters::Synchronizer<sync_policy> sync(sync_policy(20), sub_det, sub_scan);
    sync.registerCallback(boost::bind(&ClusterPublisherNode::callback, myNode, _1, _2));
    ROS_INFO("[OBJ] cluster_publisher running!");
    ros::spin();
    return 0;
}
