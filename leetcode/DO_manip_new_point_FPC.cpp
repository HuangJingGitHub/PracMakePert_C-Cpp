#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo.hpp>
#include "do_manip/visual_info_msg.h"
#include "do_manip/visual_info_srv.h"
#include "DO_manip_new_point_FPC.h"

using namespace cv;
using namespace std;

const string WINDOW = "Processing Image";

class ImageConverter {
    ros::NodeHandle nh_;
    ros::ServiceServer visual_info_service = nh_.advertiseService("visual_info_service", &ImageConverter::get_visual_info_srv, this);
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

public:
    Mat gray, prevGray;
    LK_tracker lk_tracker;
    imgExtractor extractor;
    deformJacobianPoint deformJd;
  
public:
    ImageConverter(char* ros_image_stream): it_(nh_) {
        image_sub_ = it_.subscribe(ros_image_stream, 30, &ImageConverter::imageProcess, this);
        lk_tracker = LK_tracker(WINDOW);
        extractor = imgExtractor(WINDOW);
        namedWindow(WINDOW);
    }

    ~ImageConverter() {
        destroyWindow(WINDOW);
    }

    void imageProcess(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat HSVImage;   
        cvtColor(cv_ptr->image, HSVImage, COLOR_BGR2HSV);
        GaussianBlur(HSVImage, HSVImage, Size(3, 3), 5, 5);

        cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);
        if (prevGray.empty())
            gray.copyTo(prevGray);
        lk_tracker.track(cv_ptr->image, gray, prevGray);

        extractor.extract(cv_ptr->image, HSVImage, gray, prevGray);
        if (extractor.extractSucceed) {
            extractor.effectorCharacterize();

            if (extractor.effectorCharacterizeSucceed) {
                circle(cv_ptr->image, extractor.FPC_endeffector, 5, Scalar(255, 255, 255), -1);
            }
            else if (!extractor.DOExtractSucceed)
                cout << "Deformable Object Detection Failed!\n";
            else if (!extractor.PExtractSucceed)
                cout << "End-Effector Detection Failed!\n";
            else
                cout << "DO and End-Effector Detection Failed!\n";
        }

        if (!deformJd.initialized && lk_tracker.validFeature)
            deformJd = deformJacobianPoint(extractor.FPC_endeffector, lk_tracker.pointFeature);
        else if (lk_tracker.validFeature)
            deformJd.update(extractor.FPC_endeffector, lk_tracker.pointFeature);
        if (deformJd.initialized)
            cout << "Deformation Jacobian:\n"
                << deformJd.JdCurr << "\n";
          
        // end of processing
        imshow(WINDOW, cv_ptr->image);
        waitKey(3);
  }

    bool get_visual_info_srv(do_manip::visual_info_srv::Request &req,
                            do_manip::visual_info_srv::Response &res) {
        res.featurePoint.push_back(lk_tracker.pointFeature.x);
        res.featurePoint.push_back(lk_tracker.pointFeature.y);
        res.featurePointTarget.push_back(lk_tracker.pointFeatureTarget.x);
        res.featurePointTarget.push_back(lk_tracker.pointFeatureTarget.y);

        res.deformJacobian.push_back(deformJd.JdCurr(0, 0));
        res.deformJacobian.push_back(deformJd.JdCurr(0, 1));
        res.deformJacobian.push_back(deformJd.JdCurr(1, 0));
        res.deformJacobian.push_back(deformJd.JdCurr(1, 1));
        // ROS_INFO("Successfully obtain visual information.\n");
        return true;
    }  
};

int main(int argc, char** argv) {
    if (argc == 2){
        ros::init(argc, argv, "image_processing");
        ImageConverter ic(argv[1]);
        ros::spin();
        return 0;
    } 
    else if (argc == 1) {
        // set default ros image source
        char *image_topic = (char*)"/cameras/source_camera/image";
        ros::init(argc, argv, "image_processing");
        ImageConverter ic(image_topic);
        ros::spin();
        return 1;
    }
    else {
        cout << "ERROR:\tusage - RosToOpencvImage <ros_image_topic>" << endl;    
        return 2; 
    }
}