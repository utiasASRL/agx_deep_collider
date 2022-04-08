// Author: Keenan Burnett
#include "object_tracker/object_tracker_node.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <vector>

int main(int argc, char **argv) {
    ros::init(argc, argv, "kernel_publisher");
    ros::NodeHandle nh;
    // Get parameters
    float scale, relative;
    std::string det_topic, det_topic3, det_topic4, det_topic_secondary;
    std::string det_topic_occupancy;
    XmlRpc::XmlRpcValue Q_, R_;
    std::string node_name = ros::this_node::getName();
    nh.getParam(node_name + "/scale", scale);
    nh.getParam(node_name + "/relative", relative);
    nh.getParam(node_name + "/Q", Q_);
    nh.getParam(node_name + "/R", R_);
    nh.getParam(node_name + "/det_topic", det_topic);
    nh.getParam(node_name + "/det_topic3", det_topic3);
    nh.getParam(node_name + "/det_topic4", det_topic4);
    nh.getParam(node_name + "/det_topic_secondary", det_topic_secondary);
    nh.getParam(node_name + "/det_topic_occupancy", det_topic_occupancy);
    // Publishers
    ros::Publisher det_pub = nh.advertise<zeus_msgs::Detections3D>("/Object/Detections3D", 10);
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/debug/objects", 0);
    // Subscribers
    // message_filters::Subscriber<zeus_msgs::Detections3D> sub_det(nh, det_topic, 10);
    // message_filters::Subscriber<zeus_msgs::Detections3D> sub_det3(nh, det_topic3, 10);
    // message_filters::Subscriber<zeus_msgs::Detections3D> sub_det4(nh, det_topic4, 10);
    // message_filters::Subscriber<zeus_msgs::Detections3D> sub_det_secondary(nh, det_topic_secondary, 10);
    // message_filters::Subscriber<zeus_msgs::Detections3D> sub_det_occupancy(nh, det_topic_occupancy, 10);
    // message_filters::Subscriber<nav_msgs::Odometry> sub_odom(nh, "/navsat/odom", 100);

    
    // Initialize kalman tracker node object
    KalmanTrackerNode myNode = KalmanTrackerNode(nh, det_pub, vis_pub);

    ros::Subscriber sub1 = nh.subscribe(det_topic, 10, &KalmanTrackerNode::callback, &myNode);
    ros::Subscriber sub2 = nh.subscribe(det_topic3, 10, &KalmanTrackerNode::callback, &myNode);
    ros::Subscriber sub3 = nh.subscribe(det_topic4, 10, &KalmanTrackerNode::callback, &myNode);
    ros::Subscriber sub4 = nh.subscribe(det_topic_secondary, 10, &KalmanTrackerNode::callback, &myNode);
    ros::Subscriber sub5 = nh.subscribe(det_topic_occupancy, 10, &KalmanTrackerNode::callback, &myNode);


    // Initialize kalman tracker object
    int xdim = 5;
    int ydim = 3;
    float T = 0.1;
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(xdim, xdim);      // Motion model
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(ydim, xdim);          // Observation model
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(xdim, xdim);      // Process noise
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(ydim, ydim);      // Measurement noise
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(xdim, xdim);     // Initial Covariance guess
    A(0, 3) = T;
    A(1, 4) = T;
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    for (int i = 0; i < 5; i++) {
        Q(i, i) = Q_[i];  // Process Noise
    }
    for (int i = 0; i < 3; i++) {
        R(i, i) = R_[i];  // Measurement Noise
    }
    Q = Q * scale;
    P0 = Q * 10;
    R = R * scale * relative;
    KalmanPtr kalmantracker = KalmanPtr(new kalman::KalmanTracker(A, C, Q, R, P0));
    std::vector<double> one_d_kalman_gains, point_cloud_range;
    double confidence_drop, metricGate, metric_thres, delete_time, cost_alpha, cost_beta;
    int unknown_type;
    nh.getParam(node_name + "/one_d_kalman_gains",  one_d_kalman_gains);
    nh.getParam(node_name + "/point_cloud_range",   point_cloud_range);
    nh.getParam(node_name + "/confidence_drop",     confidence_drop);
    nh.getParam(node_name + "/metricGate",          metricGate);
    nh.getParam(node_name + "/metric_thres",        metric_thres);
    nh.getParam(node_name + "/delete_time",         delete_time);
    nh.getParam(node_name + "/cost_alpha",          cost_alpha);
    nh.getParam(node_name + "/cost_beta",           cost_beta);
    nh.getParam(node_name + "/unknown_type",        unknown_type);
    kalmantracker->set1DKalmanGains(one_d_kalman_gains);
    kalmantracker->setPointCloudRange(point_cloud_range);
    kalmantracker->setConfidenceDrop(confidence_drop);
    kalmantracker->setMetricGate(metricGate);
    kalmantracker->setMetricThres(metric_thres);
    kalmantracker->setDeleteTime(delete_time);
    kalmantracker->setCostParameters(cost_alpha, cost_beta);
    kalmantracker->setUnknownType(unknown_type);
    // HMM Parameters
    std::vector<int> types;
    XmlRpc::XmlRpcValue A_, C_, pi_;
    nh.getParam(node_name + "/types",               types);
    nh.getParam(node_name + "/A_hmm",               A_);
    nh.getParam(node_name + "/C_hmm",               C_);
    nh.getParam(node_name + "/pi_hmm",              pi_);
    int num_types = types.size();
    Eigen::MatrixXd A_hmm, C_hmm, pi_hmm;
    A_hmm = Eigen::MatrixXd::Zero(num_types, num_types);
    C_hmm = Eigen::MatrixXd::Zero(num_types, num_types);
    pi_hmm = Eigen::MatrixXd::Zero(num_types, 1);
    for (int i = 0; i < num_types; i++) {
        pi_hmm(i, 0) = pi_[i];
        for (int j = 0; j < num_types; j++) {
                A_hmm(i, j) = A_[num_types * i + j];
                C_hmm(i, j) = C_[num_types * i + j];
        }
    }
    kalmantracker->setHMMParameters(types, A_hmm, C_hmm, pi_hmm);
    myNode.setKalmanTracker(kalmantracker);
    myNode.initialize_transforms();
    // typedef message_filters::sync_policies::ApproximateTime<zeus_msgs::Detections3D, nav_msgs::Odometry> sync_policy;
    // message_filters::Synchronizer<sync_policy> sync(sync_policy(100), sub_det, sub_odom);
    // message_filters::Synchronizer<sync_policy> sync3(sync_policy(100), sub_det3, sub_odom);
    // message_filters::Synchronizer<sync_policy> sync4(sync_policy(100), sub_det4, sub_odom);
    // message_filters::Synchronizer<sync_policy> sync_secondary(sync_policy(100), sub_det_secondary, sub_odom);
    // message_filters::Synchronizer<sync_policy> sync_occupancy(sync_policy(100), sub_det_occupancy, sub_odom);
    // sync.registerCallback(boost::bind(&KalmanTrackerNode::callback, myNode, _1, _2));
    // sync3.registerCallback(boost::bind(&KalmanTrackerNode::callback, myNode, _1, _2));
    // sync4.registerCallback(boost::bind(&KalmanTrackerNode::callback, myNode, _1, _2));
    // sync_secondary.registerCallback(boost::bind(&KalmanTrackerNode::callback, myNode, _1, _2));
    // sync_occupancy.registerCallback(boost::bind(&KalmanTrackerNode::callback, myNode, _1, _2));
    ROS_INFO_STREAM("[OBJ] kalman tracker running!");
    ros::spin();
    return 0;
}
