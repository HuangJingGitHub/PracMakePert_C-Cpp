#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //image process
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    std::cout << cv_ptr->image.rows << "x" << cv_ptr->image.cols << std::endl;

    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    {
        cv::circle(cv_ptr->image, cv::Point(50, 50), 20, CV_RGB(255, 0, 0));
    }
    cv::imshow("processed image", cv_ptr->image);
    //cv::imshow("image_process_view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_reciver");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("cameras/left_hand_camera/image", 1, imageCallback);
    ros::spin();
}
