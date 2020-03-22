/* 
Deformable objects manipulation_main
*/
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo.hpp>    // denoising module
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include "dvrk_retraction/visual_feedback.h"
#include "dvrk_retraction/visual_info_srv.h"
#include "DO_manip.h"

// namespace enc = sensor_msgs::image_encodings;
const std::string WINDOW = "New OpenCV Image";

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Publisher image_info_pub = nh_.advertise<dvrk_retraction::visual_feedback>("visual_info", 1000);
  ros::ServiceServer visual_info_service = nh_.advertiseService("visual_info_srv", &ImageConverter::get_visual_info_srv, this);
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  float focal_x = 683;
  float focal_y = 683;
  float depth = 58;

public:
  std::vector<float> class_centroid;
  std::vector<float> class_rotate_origin;
  std::vector<float> class_principal_axis;
  float class_rotate_angle;
  std::vector<float> class_target_region_center;
  float class_target_region_radius;
  std::vector<float> global_radius{0,0};
  int counter_radius = 0;
  cv::Mat gray, prevGray;
  LK_tracker lk_tracker;
  imgExtractor extractor;
  
public:
  ImageConverter(char* ros_image_stream): it_(nh_)
  {
    image_pub_ = it_.advertise("image_processed", 1);
    image_sub_ = it_.subscribe(ros_image_stream, 1, &ImageConverter::imageCb, this);
    lk_tracker = LK_tracker(WINDOW);
    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    namedWindow(WINDOW, 1);
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Point imageOrigin(320, 240), xAxisEnd(220, 240), yAxisEnd(320, 340);
    cv::Mat HSVImage;
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 683.9731, 0, 320., 
                                                    0, 683.9731, 240, 
                                                    0, 0, 1);
    cv::Mat distCoef = (cv::Mat_<double>(1, 5) << -0.7175, 2.8528, 0, 0, -5.1548);
    dvrk_retraction::visual_feedback image_info_msg;    
  
    cv::cvtColor(cv_ptr->image, HSVImage, COLOR_BGR2HSV);
    cv::GaussianBlur(HSVImage, HSVImage, cv::Size(3, 3), 5, 5);

    cv::cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);
    if (prevGray.empty())
      gray.copyTo(prevGray);
    lk_tracker.track(cv_ptr->image, gray, prevGray);
    lk_tracker.angle3Pts();

    extractor = imgExtractor(HSVImage);
    extractor.extract();
    if (extractor.extractSucceed){
      int DOLargestContour = extractor.DOLargestCotrIdx,
          PLargestContour = extractor.PLargestCotrIdx;
      cv::drawContours(cv_ptr->image, extractor.DOContours, DOLargestContour, cv::Scalar(255, 0, 0), 2);
      cv::drawContours(cv_ptr->image, extractor.PContours, PLargestContour, cv::Scalar(0, 255, 0), 2);
    
      extractor.effectorCharacterize();
      if (extractor.effectorCharacterizeSucceed){
        cv::circle(cv_ptr->image, extractor.endeffectorP.sl, 5, cv::Scalar(108, 37, 189), -1);
        cv::putText(cv_ptr->image, "s_l", extractor.endeffectorP.sl - cv::Point(5,5), 
                    cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0,0,0), 2);
        cv::circle(cv_ptr->image, extractor.endeffectorP.sr, 5, cv::Scalar(37, 189, 184), -1);
        cv::putText(cv_ptr->image, "s_r", extractor.endeffectorP.sr - cv::Point(5,5), 
                    cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0,0,0), 2);
        cv::Point2f neDirt(extractor.endeffectorP.ne(0), extractor.endeffectorP.ne(1));
        cv::Point2f PMid = (extractor.endeffectorP.sl + extractor.endeffectorP.sr) / 2;
        cv::circle(cv_ptr->image, PMid, 5, cv::Scalar(255, 0, 0), -1);
        cv::arrowedLine(cv_ptr->image, PMid, PMid + 60*neDirt, cv::Scalar(255, 0, 0), 2);  
      }
      else if (!extractor.DOExtractSucceed)
        std::cout << "Deformable Object Detection Failed!\n";
      else if (!extractor.PExtractSucceed)
        std::cout << "End-Effector Detection Failed!\n";
      else
        std::cout << "DO and End-Effector Detection Failed!\n";

      extractor.segment(extractor.DOContours[DOLargestContour]);
      cv::circle(cv_ptr->image, extractor.DOContours[DOLargestContour][extractor.segmentationIdx[0][1]],
                10, cv::Scalar(255, 0, 0), -1);
      cv::circle(cv_ptr->image, extractor.DOContours[DOLargestContour][extractor.segmentationIdx[0][0]],
                10, cv::Scalar(255, 0, 0), -1);                
    }
    // cv::circle(cv_ptr->image, imageOrigin, 5, cv::Scalar(0, 0, 0), -1);
    // cv::arrowedLine(cv_ptr->image, imageOrigin, xAxisEnd, cv::Scalar(0, 0, 255), 2);
    // cv::putText(cv_ptr->image, "x", xAxisEnd - cv::Point(10, 10), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255), 2);
    // cv::arrowedLine(cv_ptr->image, imageOrigin, yAxisEnd, cv::Scalar(0, 255, 0), 2);
    // cv::putText(cv_ptr->image, "y", yAxisEnd + cv::Point(10, 10), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0), 2);

    // end of processing
    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    image_info_pub.publish(image_info_msg);
  }

  bool get_visual_info_srv(dvrk_retraction::visual_info_srv::Request &req,
                          dvrk_retraction::visual_info_srv::Response &res)
  {
    res.centroid.push_back(class_centroid[0]);
    res.centroid.push_back(class_centroid[1]);
    res.rotate_origin.push_back(class_rotate_origin[0]);
    res.rotate_origin.push_back(class_rotate_origin[1]);
    res.principal_axis.push_back(class_principal_axis[0]);
    res.principal_axis.push_back(class_principal_axis[1]);
    res.rotate_angle = class_rotate_angle;
    res.target_region_center.push_back(class_target_region_center[0]);
    res.target_region_center.push_back(class_target_region_center[1]);
    res.target_region_radius = class_target_region_radius;

    // ROS_INFO("Successfully obtain visual information.\n");
    return true;
  }  
};

int main(int argc, char** argv)
{
  if (argc == 2){
    ros::init(argc, argv, "image_processing");
    ImageConverter ic(argv[1]);
    ros::spin();
    return 0;
  } 
  else if (argc == 1){
  // set default ros image source
    char *image_topic = (char*) "/cameras/source_camera/image";
    ros::init(argc, argv, "image_processing");
    ImageConverter ic(image_topic);
    ros::spin();
    return 1;
  }
  else{
    std::cout << "ERROR:\tusage - RosToOpencvImage <ros_image_topic>" << std::endl;    
    return 2; 
  }
}