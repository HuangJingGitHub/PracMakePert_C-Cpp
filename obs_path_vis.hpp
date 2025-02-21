#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "../3d/obstacles_3d.hpp"

void VisualizeObstaclesAndPath(vector<PolygonObstacle3d> obs_vec, vector<Point3f>& path) {
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("obstacle_marker_array", 10);
    visualization_msgs::MarkerArray marker_array;
    int obs_num = obstacles.size();

    for (int i = 4; i < obs_num + 4; i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/my_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
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
        marker.pose.orientation.z = 1.0;
        marker.pose.orientation.w = cos(angle / 2);

        marker.scale.x = length;
        marker.scale.y = width;
        marker.scale.z = obs_vec[i].height;

        marker.color.r = 0.8275f;
        marker.color.g = 0.8275f;
        marker.color.b = 0.8275f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        marker_array.markers.push_back(marker);
    }  

    while (ros::ok()) {
        // Publish the marker
        while (marker_array_pub.getNumSubscribers() < 1)
        {
        if (!ros::ok())
        {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
        }
        marker_array_pub.publish(marker_array);
        r.sleep();
    }
}