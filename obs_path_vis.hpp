#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "../3d/obstacles_3d.hpp"

void VisualizeObstaclesAndPath(const vector<PolygonObstacle3d>& obs_vec, const vector<Point3f>& path = {}) {
    ros::NodeHandle obs_node, path_node;
    ros::Rate update_rate(1);
    ros::Publisher obs_array_pub = obs_node.advertise<visualization_msgs::MarkerArray>("obstacle_marker_array", 10),
                   path_marker_pub = path_node.advertise<visualization_msgs::Marker>("path_marker", 10);
    visualization_msgs::MarkerArray obs_array;
    visualization_msgs::Marker path_marker;
    
    int obs_num = obs_vec.size() - 4, path_node_num = path.size();
    obs_array.markers = vector<visualization_msgs::Marker>(obs_num);
    path_marker.points = vector<geometry_msgs::Point>(path_node_num);
    path_marker.colors = vector<std_msgs::ColorRGBA>(path_node_num);
    string frame_id = "planning_frame";
    // Skip environment walls
    for (int i = 4; i < obs_num + 4; i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "obstacle_shapes";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        Point2f obs_centroid(0, 0);
        for (int j = 0; j < 4; j++)
            obs_centroid += obs_vec[i].vertices[j];
        obs_centroid /= 4.0;
        Point2f bottom_side = obs_vec[i].vertices[1] - obs_vec[i].vertices[0];
        float length = cv::norm(obs_vec[i].vertices[1] - obs_vec[i].vertices[0]),
              width = cv::norm(obs_vec[i].vertices[2] - obs_vec[i].vertices[1]),
              angle = atan2(bottom_side.y, bottom_side.x);

        marker.pose.position.x = obs_centroid.x;
        marker.pose.position.y = obs_centroid.y;
        marker.pose.position.z = obs_vec[i].height / 2;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = sin(angle / 2);
        marker.pose.orientation.w = cos(angle / 2);

        marker.scale.x = length;
        marker.scale.y = width;
        marker.scale.z = obs_vec[i].height;

        marker.color.r = 0.8275f;
        marker.color.g = 0.8275f;
        marker.color.b = 0.8275f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        obs_array.markers[i - 4] = marker;
    }  

    path_marker.header.frame_id = frame_id;
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "path_points";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;   
    path_marker.pose.orientation.x = 0.0;
    path_marker.pose.orientation.y = 0.0;
    path_marker.pose.orientation.z = 0.0;
    path_marker.pose.orientation.w = 1.0; 
    path_marker.scale.x = 1;
    for (int i = 0; i < path_node_num; i++) {
        geometry_msgs::Point point;
        point.x = path[i].x;
        point.y = path[i].y;
        point.z = path[i].z;
        path_marker.points[i] = point;

        std_msgs::ColorRGBA rgba;
        rgba.r = 0;
        rgba.g = 0;
        rgba.b = 1.0;
        rgba.a = 1.0;
        path_marker.colors[i] = rgba;
    }      

    while (ros::ok()) {
        // Publish the marker
        while (obs_array_pub.getNumSubscribers() < 1) {
        if (ros::ok() == false) {
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
        }
        obs_array_pub.publish(obs_array);
        path_marker_pub.publish(path_marker);
        update_rate.sleep();
    }
}