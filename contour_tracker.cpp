/* 

    HUANG Jing
    haungjing@mae.cuhk.edu.hk
    Partially based on correll lab's script http://104.236.202.16/?p=3064.
    Track red feature shape contour in an image.

*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "New OpenCV Image";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    float focal_x = 683;
    float focal_y = 683;
    float depth = 58;
  
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

   // Find countors of red shape
    cv::Point image_origin(320, 240),
              x_axis_end(220, 240), y_axis_end(320, 340);
    cv::Scalar red_low(0, 173, 152), red_high(10, 255, 255);
    std::vector< std::vector<cv::Point>> contours;
    cv::Mat tracked_image, dst, tracked_image_undist;
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 683.9731, 0, 320., 0, 683.9731, 240, 0, 0, 1);
    cv::Mat dist_coef = (cv::Mat_<double>(1, 5) << -0.7175, 2.8528, 0, 0, -5.1548);
    
    cv::cvtColor(cv_ptr->image, tracked_image, CV_BGR2HSV);
    cv::inRange(tracked_image, red_low, red_high, dst);
    cv::imshow("Binary Image", dst);

    /*  Use bgr difference to generate binary image rather than inRange()
    cv::Mat bgr_thr = cv::Mat::zeros(tracked_image.size(), CV_8UC1);
    int Rows = bgr_thr.rows, Cols = bgr_thr.cols;
    for (int r = 0; r < Rows; r ++)
        for (int c = 0; c < Cols; c++)
        {
            cv::Vec3b intensity = cv_ptr->image.at<cv::Vec3b>(cv::Point(c, r));
            if (intensity[2] - intensity[0] > 100 && intensity[2] - intensity[1] > 100)
            {
                bgr_thr.at<uchar>(cv::Point(c, r)) = 255;
            }
        }
    */
   
    cv::Moments m_dst = moments(dst, true);
    cv::Point p(m_dst.m10/m_dst.m00, m_dst.m01/m_dst.m00);

    if (p.x < 0 || p.y < 0)
    {
      std::cout << "No feature shape detected." << std::endl;
    }
    else
    {
      // cv::findContours(bgr_thr, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      cv::findContours(dst, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      cv::drawContours(cv_ptr->image, contours, -1, cv::Scalar(255, 0, 0), 2);
    }

    cv::circle(cv_ptr->image, image_origin, 5, cv::Scalar(0, 0, 0), -1);
    cv::arrowedLine(cv_ptr->image, image_origin, x_axis_end, cv::Scalar(0, 0, 255), 2);
    cv::putText(cv_ptr->image, "x", x_axis_end - cv::Point(10, 10), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
    cv::arrowedLine(cv_ptr->image, image_origin, y_axis_end, cv::Scalar(0, 255, 0), 2);
    cv::putText(cv_ptr->image, "y", y_axis_end + cv::Point(10, 10), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0), 2);

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
    ros::init(argc, argv, "image_processing");
    ImageConverter ic(argv[1]);
    ros::spin();
    return 0;
  } 
  else if (argc == 1)
  {
  // set default ros image source， HUANG Jing
    char *image_topic = (char*) "/cameras/source_camera/image";
    ros::init(argc, argv, "image_processing");
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
