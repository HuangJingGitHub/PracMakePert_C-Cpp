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
#include <opencv2/photo.hpp>    // denoising module
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <vector>
#include <math.h>

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
    cv::Point imageOrigin(320, 240),
              xAxisEnd(220, 240), yAxisEnd(320, 340);
    cv::Scalar redLow(0, 173, 152), redHigh(10, 255, 255), black_low(0, 0, 0), black_high(149, 218, 71);
    std::vector< std::vector<cv::Point>> contours;
    cv::Mat trackedImage, trackedImage_denoised, dst, trackedImage_undist;
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 683.9731, 0, 320., 0, 683.9731, 240, 0, 0, 1);
    cv::Mat distCoef = (cv::Mat_<double>(1, 5) << -0.7175, 2.8528, 0, 0, -5.1548);
    
    cv::cvtColor(cv_ptr->image, trackedImage, CV_BGR2HSV);
    // cv::fastNlMeansDenoisingColored(trackedImage,trackedImage_denoised, 3, 3, 7, 21); // This function costs much time.
    cv::blur(trackedImage, trackedImage, cv::Size(3, 3));   // blurring is very important to get rid of noise.
    cv::inRange(trackedImage, black_low, black_high, dst);
    cv::imshow("Binary Image", dst);

    /* Use bgr difference to generate binary image rather than inRange()
    cv::Mat bgr_thr = cv::Mat::zeros(trackedImage.size(), CV_8UC1);
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
    cv::Point p(m_dst.m10 / m_dst.m00, m_dst.m01 / m_dst.m00);

    if (p.x < 0 || p.y < 0)
    {
      std::cout << "No feature shape detected." << std::endl;
    }
    else
    {
      // cv::findContours(bgr_thr, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      cv::findContours(dst, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      int largestContourIndex = 0;
      for (int i = 0; i < contours.size(); i++)
        largestContourIndex = (contours[i].size() > contours[largestContourIndex].size()) ? i : largestContourIndex;
      cv::Moments shapeMoments = moments(contours[largestContourIndex]);
      cv::Point2f shapeCenter(static_cast<float>(shapeMoments.m10 / shapeMoments.m00),
                              static_cast<float>(shapeMoments.m01 / shapeMoments.m00));
      Eigen::Matrix2f I;
      I <<  shapeMoments.mu20, shapeMoments.mu11,
            shapeMoments.mu11, shapeMoments.mu02;
      Eigen::EigenSolver<Eigen::Matrix2f> es(I);
      Eigen::Vector2d eigenVector1(es.eigenvectors().col(0).real()[0], es.eigenvectors().col(0).real()[1]),
                      eigenVector2(es.eigenvectors().col(1).real()[0], es.eigenvectors().col(1).real()[1]);

      // ***The procedure is to get rid of the possible random switch of the principal axis display.
      // ***There is a need of look at the characters of the retrun eigenvectors first to know how to
      // ***better set the processing.
      if (eigenVector1[0] < 0 )
      {
        Eigen::Vector2d tempVector = eigenVector1;
        eigenVector1 = eigenVector2;
      } 
      eigenVector2 = Eigen::Vector2d(eigenVector1[1], -eigenVector1[0]);
      /* float angleX1 = atan2(eigenVector1[1], eigenVector1[0]),
            angleX2 = atan2(eigenVector2[1], eigenVector2[0]);
      if (angleX1 > angleX2)
      {
        Eigen::Vector2d tempVector = eigenVector1;
        eigenVector1 = eigenVector2;
        eigenVector2 = tempVector;
      } */
      // std::cout << es.eigenvectors().col(0).real()[0] << "*****" << std::endl;
      cv::Point2f principleAxie1 = shapeCenter + 80 * cv::Point2f( eigenVector1[0], eigenVector1[1]),
                  principleAxie2 = shapeCenter + 80 * cv::Point2f( eigenVector2[0], eigenVector2[1]);      
      cv::arrowedLine(cv_ptr->image, shapeCenter, principleAxie1, cv::Scalar(0, 0, 255), 2);
      cv::arrowedLine(cv_ptr->image, shapeCenter, principleAxie2, cv::Scalar(0, 255, 0), 2);
      cv::drawContours(cv_ptr->image, contours, largestContourIndex, cv::Scalar(255, 0, 0), 2);
    }

    cv::circle(cv_ptr->image, imageOrigin, 5, cv::Scalar(0, 0, 0), -1);
    cv::arrowedLine(cv_ptr->image, imageOrigin, xAxisEnd, cv::Scalar(0, 0, 255), 2);
    cv::putText(cv_ptr->image, "x", xAxisEnd - cv::Point(10, 10), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
    cv::arrowedLine(cv_ptr->image, imageOrigin, yAxisEnd, cv::Scalar(0, 255, 0), 2);
    cv::putText(cv_ptr->image, "y", yAxisEnd + cv::Point(10, 10), CV_FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0), 2);
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
