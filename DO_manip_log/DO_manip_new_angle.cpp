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
#include <opencv2/photo.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include "do_manip/visual_info_msg.h"
#include "do_manip/visual_info_srv.h"
#include "DO_manip_new_angle.h"

using namespace cv;
using namespace std;

const string WINDOW = "New OpenCV Image";

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::ServiceServer visual_info_service = nh_.advertiseService("visual_info_service", &ImageConverter::get_visual_info_srv, this);
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  Mat gray, prevGray;
  LK_tracker lk_tracker;
  imgExtractor extractor;
  optConstructor opt;
  deformJacobianAngle deformJd;
  angleFeature3Pts featureAngle;
  
public:
  ImageConverter(char* ros_image_stream): it_(nh_)
  {
    image_sub_ = it_.subscribe(ros_image_stream, 30, &ImageConverter::imageProcess, this);
    lk_tracker = LK_tracker(WINDOW);
    extractor = imgExtractor(WINDOW);
    namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(WINDOW);
  }

  void imageProcess(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat HSVImage;   
    cvtColor(cv_ptr->image, HSVImage, COLOR_BGR2HSV);
    GaussianBlur(HSVImage, HSVImage, Size(3, 3), 5, 5);
    do_manip::visual_info_msg image_info_msg; 

    cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);
    if (prevGray.empty())
      gray.copyTo(prevGray);
    lk_tracker.track(cv_ptr->image, gray, prevGray);

    extractor.extract(cv_ptr->image, HSVImage, gray, prevGray);
    if (extractor.extractSucceed){
      drawContours(cv_ptr->image, extractor.DOContours, extractor.DOLargestCotrIdx, Scalar(255, 0, 0), 2);
      extractor.effectorCharacterize();

      if (extractor.effectorCharacterizeSucceed){
        putText(cv_ptr->image, "s_l", extractor.endeffectorP.sl - Point(5,5), 
                FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,0), 2);
        putText(cv_ptr->image, "s_r", extractor.endeffectorP.sr - Point(5,5), 
                FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,0), 2);
        Point2f neDirt(extractor.endeffectorP.ne(0), extractor.endeffectorP.ne(1));
        Point2f PMid = (extractor.endeffectorP.sl + extractor.endeffectorP.sr) / 2;
        circle(cv_ptr->image, PMid, 5, Scalar(255, 0, 0), -1);
        arrowedLine(cv_ptr->image, PMid, PMid + 60*neDirt, Scalar(255, 0, 0), 2);  
      }
      else if (!extractor.DOExtractSucceed)
        cout << "Deformable Object Detection Failed!\n";
      else if (!extractor.PExtractSucceed)
        cout << "End-Effector Detection Failed!\n";
      else
        cout << "DO and End-Effector Detection Failed!\n";
      
      if (extractor.segment(extractor.DOContour)){
        circle(cv_ptr->image, extractor.DOContour[extractor.segmentationIdx[0][0]],
                  5, Scalar(255, 0, 0), -1);
        circle(cv_ptr->image, extractor.DOContour[extractor.segmentationIdx[0][1]],
                  5, Scalar(255, 0, 0), -1); 
        circle(cv_ptr->image, extractor.DOContour[extractor.segmentationIdx[1][0]],
                  5, Scalar(255, 0, 0), -1);
        circle(cv_ptr->image, extractor.DOContour[extractor.segmentationIdx[1][1]],
                  5, Scalar(255, 0, 0), -1); 
        line(cv_ptr->image, extractor.DOContour[extractor.segmentationIdx[0][0]],
                extractor.DOContour[extractor.segmentationIdx[0][1]], Scalar(0,0,255), 2); 
        line(cv_ptr->image, extractor.DOContour[extractor.segmentationIdx[1][0]],
                extractor.DOContour[extractor.segmentationIdx[1][1]], Scalar(0,0,255), 2);                     
      }
      // vector<Point> newP = extractor.feaibleMotionSearch();
      // line(cv_ptr->image, newP[0], newP[1], Scalar(32,54,134), 2);

      featureAngle = lk_tracker.angle3Pts();
      opt = optConstructor(extractor);
      // opt.getDeformConstraint();
      if (lk_tracker.validFeature) 
        opt.getManipulabilityAngle(featureAngle);
    }

    if (!deformJd.initialized && lk_tracker.validFeature)
      deformJd = deformJacobianAngle(extractor.endeffectorP, featureAngle);
    else if (lk_tracker.validFeature)
      deformJd.update(extractor.endeffectorP, featureAngle);
    if (deformJd.initialized)
      cout << "Deformation Jacobian:\n"
          << deformJd.JdCurr << "\n";
          
    // end of processing
    imshow(WINDOW, cv_ptr->image);
    waitKey(3);
  }

  bool get_visual_info_srv(do_manip::visual_info_srv::Request &req,
                          do_manip::visual_info_srv::Response &res)
  {
    res.sl.push_back(extractor.endeffectorP.sl.x);
    res.sl.push_back(extractor.endeffectorP.sl.y);
    res.sr.push_back(extractor.endeffectorP.sr.x);
    res.sr.push_back(extractor.endeffectorP.sr.y);
    res.ne.push_back(extractor.endeffectorP.ne[0]);
    res.ne.push_back(extractor.endeffectorP.ne[1]);
    
    res.contactProjectionl.push_back(extractor.DOContour[extractor.segmentationIdx[0][0]].x);
    res.contactProjectionl.push_back(extractor.DOContour[extractor.segmentationIdx[0][0]].y);
    res.contactProjectionr.push_back(extractor.DOContour[extractor.segmentationIdx[1][0]].x);
    res.contactProjectionr.push_back(extractor.DOContour[extractor.segmentationIdx[1][0]].y);
    res.contactDistancelr.push_back(extractor.contactDistances[0]);
    res.contactDistancelr.push_back(extractor.contactDistances[1]);
    //res.deformAngles.push_back(opt.deformAngles[0]);
    //res.deformAngles.push_back(opt.deformAngles[1]);

    //res.featurePoint.push_back(lk_tracker.pointFeature.x);
    //res.featurePoint.push_back(lk_tracker.pointFeature.y);
    //res.featurePointTarget.push_back(lk_tracker.pointFeatureTarget.x);
    //res.featurePointTarget.push_back(lk_tracker.pointFeatureTarget.y);

    res.sw.push_back(opt.sw.x);
    res.sw.push_back(opt.sw.y);
    res.indicatorw = opt.indicatorw;
    res.distancew = opt.distancew;
    res.distancelrp.push_back(opt.distanceLinelrp[0]);
    res.distancelrp.push_back(opt.distanceLinelrp[1]);
    res.distancelrp.push_back(opt.distanceLinelrp[2]);

    res.featureAngley = featureAngle.angle;
    res.deformJacobian.push_back(deformJd.JdCurr(0, 0));
    res.deformJacobian.push_back(deformJd.JdCurr(0, 1));
    //res.deformJacobian.push_back(deformJd.JdCurr(1, 0));
    //res.deformJacobian.push_back(deformJd.JdCurr(1, 1));
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
    char *image_topic = (char*)"/cameras/source_camera/image";
    ros::init(argc, argv, "image_processing");
    ImageConverter ic(image_topic);
    ros::spin();
    return 1;
  }
  else{
    cout << "ERROR:\tusage - RosToOpencvImage <ros_image_topic>" << endl;    
    return 2; 
  }
}