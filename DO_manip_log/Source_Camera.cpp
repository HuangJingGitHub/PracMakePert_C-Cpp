#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <sstream>


using namespace std;
using namespace cv;

class webcam_test{
public:
    webcam_test()
    {
        VideoCapture monocam(0);

        cout <<  "This is the 2D view!\n";
        namedWindow("Source Video Stream", WINDOW_AUTOSIZE);
        if (!monocam.isOpened()){	// if not success, exit program
            cout <<  "Cannot open the Webcam" << endl;
            return;
        }

        double dWidth_l  = monocam.get(cv::CAP_PROP_FRAME_WIDTH); 
        double dHeight_l = monocam.get(cv::CAP_PROP_FRAME_HEIGHT); 
        cout << "Frame size of camera: " << dWidth_l << " x " << dHeight_l << endl;

        //cout << "Frame rate of LEFT cam: " << monocam.get(CV_CAP_PROP_MODE) << endl;
        monocam.set(CAP_PROP_FPS, 30);
        int nFPS_l = monocam.get(CAP_PROP_FPS);
        cout << "Frame rate of monocamera: "  << nFPS_l << endl;
        // namedWindow("Source Video Stream", WINDOW_NORMAL); // manually resize the window

        ros::NodeHandle nh;
        image_transport::ImageTransport it_left(nh);
        image_transport::Publisher publ = it_left.advertise("cameras/source_camera/image", 200);
        sensor_msgs::ImagePtr msg_l;
        ros::Rate publish_rate(30);

        Mat frame_cam;
        while (true){
            bool leftSuccess = monocam.read(frame_cam);   

            if (!leftSuccess) {
                cout <<  "ERROR: Cannot read a frame from the video stream!\n";
                break;
            }
            // imshow("Source Video Stream", frame_cam);

            msg_l = cv_bridge::CvImage( std_msgs::Header(), "bgr8", frame_cam).toImageMsg();
            if (ros::ok()){
                publ.publish(msg_l);
                publish_rate.sleep();
            }
            else{
                monocam.release();
                cout << "\nTerminated by user.\n";
                break;
            }         
        }

    }

    ~webcam_test(){
        destroyWindow("Source Video Stream");  // Destroy window
    }
};

class stereo_test{
private:
    char exit_key_press;
public:
    stereo_test(){
        VideoCapture capLeft(0);
        VideoCapture capRight(2);

        cout <<  "Stereo view from both scopes" << endl;

        if (!capLeft.isOpened() || !capRight.isOpened() ){	// if not success, exit program
            cout <<  "Cannot open the Webcams" << endl;
            return;

        }else if (!capRight.isOpened() && capLeft.isOpened()){
            cout <<  "Cannot open the right scope" << endl;
            return;

        }else if (capRight.isOpened() && !capLeft.isOpened()){
            cout <<  "Cannot open the left scope" << endl;
            return;

        }else if (capRight.isOpened() && capLeft.isOpened()){

        /*capLeft.set(CAP_PROP_FRAME_WIDTH,640);
        capLeft.set(CAP_PROP_FRAME_HEIGHT,480);
        capRight.set(CAP_PROP_FRAME_WIDTH,640);
        capRight.set(CAP_PROP_FRAME_HEIGHT,480);*/

            double dWidth_l  = capLeft.get(CAP_PROP_FRAME_WIDTH); 
            double dHeight_l = capLeft.get(CAP_PROP_FRAME_HEIGHT); 
            cout << "Frame size of left cam: " << dWidth_l << " x " << dHeight_l << endl;

            capLeft.set(CAP_PROP_FPS,30);
            int nFPS_l = capLeft.get(CAP_PROP_FPS);
            cout << "Frame rate of left cam: "  << nFPS_l << endl;

            namedWindow("Left Scope Video Stream", WINDOW_AUTOSIZE); 

            double Width_right  = capRight.get(CAP_PROP_FRAME_WIDTH);
            double Height_right = capLeft.get(CAP_PROP_FRAME_HEIGHT);
            cout << "Frame size of right cam: " << Width_right << " x " << Height_right << endl;

            capRight.set(CAP_PROP_FPS,30);
            int nFPS_r = capRight.get(CAP_PROP_FPS);
            cout << "Frame rate of right cam::" << nFPS_r << endl;
            namedWindow("Right Scope Video Stream", WINDOW_AUTOSIZE);
        }
        ros::NodeHandle nh;
        image_transport::ImageTransport it_left(nh);
        image_transport::Publisher publ = it_left.advertise("cameras/left_hand_camera/image", 100);
        image_transport::ImageTransport it_right(nh);
        image_transport::Publisher pubr = it_right.advertise("cameras/right_hand_camera/image", 100);

        Mat frame_left;
        Mat frame_right;

        while (true){
            bool leftSuccess = capLeft.read(frame_left);   // read a new frame from the video
            bool rightSuccess = capRight.read(frame_right);
            if (!leftSuccess || !rightSuccess){            // if not success, break loop
                cout <<  "ERROR: Cannot read from scopes!" << endl;
                break;
            }

            imshow("Left Scope Video Stream", frame_left);
            imshow("Right Scope Video Stream", frame_right);

            //cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
            //frame_msg = frame;
            sensor_msgs::ImagePtr msg_l = cv_bridge::CvImage( std_msgs::Header(), "bgr8", frame_left).toImageMsg();
            sensor_msgs::ImagePtr msg_r = cv_bridge::CvImage( std_msgs::Header(), "bgr8", frame_right).toImageMsg();

            if (!ros::ok()){
                cout <<  "\nTerminated by user" << endl;
                break;
            }

            publ.publish(msg_l);
            pubr.publish(msg_r);
            waitKey(30); // for sync
        }

        capLeft.release();
        capRight.release();
    }

    ~stereo_test(){
        destroyWindow("Left Scope Video Stream");  // Destroy window
        destroyWindow("Right Scope Video Stream");
    }
};


int main (int argc, char **argv) 
{
    ros::init(argc, argv, "Source_Camera");

    cout << argc << " Arguements" << endl;
    if (argc == 2 ) {
        webcam_test webcam_object;
    } else if (argc == 3 ) {
        stereo_test stereo_object;
    } else {
        ROS_INFO("ERROR: Unrecognized arguements");
        return 1;
    }

}
