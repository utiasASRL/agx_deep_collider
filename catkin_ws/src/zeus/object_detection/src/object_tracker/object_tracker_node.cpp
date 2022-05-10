// Author: Keenan Burnett
#include "object_tracker/object_tracker_node.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

// Given an object list, formats the appropriate ROS output message for object detection
void convertToRosMessage(const std::vector<Object> &object_list, zeus_msgs::Detections3D &outputDetections, float yaw) {
    for (int i = 0; i < (int)object_list.size(); i++) {
        zeus_msgs::BoundingBox3D detection;
        detection.x = object_list[i].x_hat(0, 0);
        detection.y = object_list[i].x_hat(1, 0);
        detection.z = object_list[i].x_hat(2, 0);
        detection.x_dot = object_list[i].x_hat(3, 0);
        detection.y_dot = object_list[i].x_hat(4, 0);
        detection.w = object_list[i].w;
        detection.l = object_list[i].l;
        detection.h = object_list[i].h;
        detection.yaw = object_list[i].yaw + yaw;
        detection.ID = object_list[i].ID;
        detection.type = object_list[i].type;
        detection.camera = object_list[i].camera;
        detection.confidence = object_list[i].confidence;
        outputDetections.bbs.push_back(detection);
    }
}

void KalmanTrackerNode::visualize(const std::vector<Object> &object_list, ros::Time stamp, float yaw) {
    visualization_msgs::MarkerArray marray;
    
    for (int i = 0; i < (int)object_list.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = stamp;
        marker.ns = "detector";
        marker.pose.position.x = object_list[i].x_hat(0, 0);
        marker.pose.position.y = object_list[i].x_hat(1, 0);
        marker.pose.position.z = object_list[i].x_hat(2, 0);
        marker.id = object_list[i].ID;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = object_list[i].l;
        marker.scale.y = object_list[i].w;
        marker.scale.z = object_list[i].h;
        marker.color.a = 0.5;
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
        // Orientation
        float theta = object_list[i].yaw + yaw;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = sin(theta / 2);
        marker.pose.orientation.w = cos(theta / 2);
        marker.lifetime = ros::Duration(1.0);
        marray.markers.push_back(marker);

        marker = visualization_msgs::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = stamp;
        marker.ns = "detector";
        marker.id = object_list[i].ID * -1;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.color.a = 0.5;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.25;
        // marker.scale.z = object_list[i].h;
        geometry_msgs::Point p;
        p.x = object_list[i].x_hat(0, 0);
        p.y = object_list[i].x_hat(1, 0);
        p.z = object_list[i].x_hat(2, 0);
        marker.points.push_back(p);
        p.x += object_list[i].x_hat(3, 0);
        p.y += object_list[i].x_hat(4, 0);
        marker.points.push_back(p);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(1.0);
        marray.markers.push_back(marker);
    }
    vis_pub_.publish(marray);
}

void KalmanTrackerNode::callback(const zeus_msgs::Detections3D::ConstPtr & det) {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<zeus_msgs::BoundingBox3D> dets = det->bbs;
    zeus_tf::get_transform(tfBuffer, "velodyne", "map", Tmv);
    // Eigen::Matrix4d Toi = Eigen::Matrix4d::Identity();
    // zeus_tf::get_odom_tf(*odom, Toi);
    // Eigen::Matrix4d Toc = Toi * Tic2;
    kalmantracker->setCurrentTime(det->header.stamp.toSec());
    //! Perform data association between the new detections (dets) and the existing object tracks
    kalmantracker->association(dets, Tmv);
    //! Linear Kalman filter update
    kalmantracker->filter(dets, Tmv);
    //! Prune objects that are closer than metricGate to each other or objects outside point_cloud_range.
    kalmantracker->prune(zeus_tf::get_inverse_tf(Tmv));
    //! Publish ROS message:
    zeus_msgs::Detections3D outputDetections;
    outputDetections.header.stamp = det->header.stamp;
    outputDetections.header.frame_id = world_frame_id;
    outputDetections.bbs.clear();
    std::vector<Object> object_list = kalmantracker->get_object_list();
    std::vector<double> rpy;
    // zeus_tf::get_rpy_from_odom(*odom, rpy);
    zeus_tf::rot_to_rpy(Tmv.block(0, 0, 3, 3), rpy);
    convertToRosMessage(object_list, outputDetections, (float)rpy[2]);
    det_pub_.publish(outputDetections);
    visualize(object_list, det->header.stamp, (float)rpy[2]);
    auto stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = stop - start;
    ROS_DEBUG_STREAM("[OBJ] KALMAN TRACKER TIME: " << elapsed.count());
}

void KalmanTrackerNode::initialize_transforms() {
    tfBuffer = zeus_tf::tfBufferPtr(new zeus_tf::tfBuffer());
    tfListener = zeus_tf::tfListenerPtr(new zeus_tf::tfListener(*tfBuffer));
    // zeus_tf::get_transform(tfBuffer, "velodyne", "map", Tmv);
    ROS_INFO_STREAM("[OBJ] object_tracker transforms initialized");
}


