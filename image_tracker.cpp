/* 

    HUANG Jing
    haungjing@mae.cuhk.edu.hk
    Based on correll lab's script http://104.236.202.16/?p=3064.
    Track red feature point in an image.

*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "OpenCV Image";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
  
public:
    ImageConverter(char* ros_image_stream): it_(nh_)
    {
        image_pub_ = it_.advertise("correll_ros2opencv", 1);
        image_sub_ = it_.subscribe(ros_image_stream, 1, &ImageConverter::imageCb, this);

        cv::namedWindow(WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    
 /* Add any OpenCV processing here */
 /* Gray scale image */
 /* 
    cv::Mat filtered_image;
    cv::cvtColor(cv_ptr->image, filtered_image, CV_BGR2GRAY);
    
    // Image Filtering 
    cv::Mat element5(5, 5, CV_8U, cv::Scalar(1));
    cv::morphologyEx(filtered_image, filtered_image, cv::MORPH_CLOSE,element5);
    cv::morphologyEx(filtered_image, filtered_image, cv::MORPH_OPEN,element5);

    //cv::GaussianBlur(filtered_image,filtered_image,cv::Size(5,5),1.5);

    cv::blur(filtered_image, filtered_image, cv::Size(3,3));    

    // Show final filtered image 
    cv::namedWindow("Filtered Image");
    cv::imshow("Filtered Image", filtered_image); 
*/

   // Find red feature point 
    cv::Point goal_point(100, 100);
    cv::Scalar red_low(0, 173, 152), red_high(10, 255, 255);
    cv::Mat tracked_image, dst;
    cv::cvtColor(cv_ptr->image, tracked_image, CV_BGR2HSV);
    cv::inRange(tracked_image, red_low, red_high, dst);
    cv::imshow("Feature Point Track", dst);

    cv::Moments m_dst = moments(dst, true);
    cv::Point p(m_dst.m10/m_dst.m00, m_dst.m01/m_dst.m00);

    if (p.x < 0 || p.y < 0)
    {
      std::cout << "No feature point detected." << std::endl;
    }
    else
    {
      cv::circle(cv_ptr->image, p, 10, cv::Scalar(255, 0, 0), -1);

      std::cout << "centroid: " << p << std::endl
                << "goal " << goal_point << std::endl
                << "image_size: " << tracked_image.size  << std::endl; 
      std::cout << "Error piex: " << goal_point - p << std::endl;
    }
    cv::circle(cv_ptr->image, goal_point, 10, cv::Scalar(0, 0, 255), -1);
    cv::putText(cv_ptr->image, "Goal", goal_point, CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 0, 0), 2);
   // end of processing
    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());   
    }
  
};

int main(int argc, char** argv)
{
  if (argc == 2) 
  {
    ros::init(argc, argv, "correll_image_converter");
    ImageConverter ic(argv[1]);
    ros::spin();
    return 0;
  } 
  else if (argc == 1)
  {
  // set default ros image source， HUANG Jing
    char *image_topic = (char*) "/cameras/left_hand_camera/image";
    ros::init(argc, argv, "correll_image_converter");
    ImageConverter ic(image_topic);
    ros::spin();
    return 1;
  }
  else
  {
    std::cout << "ERROR:\tusage - RosToOpencvImage <ros_image_topic>" << std::endl; 
    return 1;    
  }
}
