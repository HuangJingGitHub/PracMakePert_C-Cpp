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
        VideoCapture monocam(1);

        cout <<  "This is the 2D view!" << endl;

        if (!monocam.isOpened()){	// if not success, exit program
            cout <<  "Cannot open the Webcam" << endl;
            return;
        }

        double dWidth_l  = monocam.get(cv::CAP_PROP_FRAME_WIDTH); // get the width of frames of the video
        double dHeight_l = monocam.get(cv::CAP_PROP_FRAME_HEIGHT); // get the height of frames of the video
        cout << "Frame size of cam: " << dWidth_l << " x " << dHeight_l << endl;

//      cout << "Frame rate of LEFT cam: " << monocam.get(CV_CAP_PROP_MODE) << endl;
        monocam.set(CAP_PROP_FPS, 30);
        int nFPS_l = monocam.get(CAP_PROP_FPS);
        cout << "Frame rate of LEFT cam: "  << nFPS_l << endl;
        // namedWindow("Source Video Stream", WINDOW_NORMAL); // manually resize the window
        namedWindow("Source Video Stream", WINDOW_AUTOSIZE);

        ros::NodeHandle nh;
        image_transport::ImageTransport it_left(nh);
        image_transport::Publisher publ = it_left.advertise("cameras/source_camera/image", 1000);
        ros::Rate publish_rate(30);

        Mat frame_cam;
        while (1)
        {
            bool leftSuccess = monocam.read(frame_cam);   

            if (!leftSuccess) 
            {
                cout <<  "ERROR: Cannot read a frame from the video stream!" << endl;
                break;
            }

            //imshow("Source Video Stream", frame_cam);

            sensor_msgs::ImagePtr msg_l = cv_bridge::CvImage( std_msgs::Header(), "bgr8", frame_cam).toImageMsg();

            //if (waitKey(40) == 27 || !(ros::ok()) )     // !(ros::ok()) added by HUNAG Jing. Wait of 40 ms, publish at 25Hz
            //{
            //    cout <<  "\nESC is pressed by user or ROS not ok." << endl;
            //    break;
            //}*/
            if (ros::ok())
            {
                publ.publish(msg_l);
                publish_rate.sleep();
            }
            else
                break;
            
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
        VideoCapture capleft(0);
        VideoCapture capright(1);

        cout <<  "Stereo view from both scopes" << endl;

        if (!capleft.isOpened() || !capright.isOpened() ){	// if not success, exit program
            cout <<  "Cannot open the Webcams" << endl;
            return;

        }else if (!capright.isOpened() && capleft.isOpened()){
            cout <<  "Cannot open the right scope" << endl;
            return;

        }else if (capright.isOpened() && !capleft.isOpened()){
            cout <<  "Cannot open the left scope" << endl;
            return;

        }else if (capright.isOpened() && capleft.isOpened()){

            capleft.set(CAP_PROP_FRAME_WIDTH,640);
            capleft.set(CAP_PROP_FRAME_HEIGHT,480);

            capright.set(CAP_PROP_FRAME_WIDTH,640);
            capright.set(CAP_PROP_FRAME_HEIGHT,480);

            double dWidth_l  = capleft.get(CAP_PROP_FRAME_WIDTH); // get the width of frames of the video
            double dHeight_l = capleft.get(CAP_PROP_FRAME_HEIGHT); // get the height of frames of the video
            cout << "Frame size of left cam: " << dWidth_l << " x " << dHeight_l << endl;

            capleft.set(CAP_PROP_FPS,10);
            int nFPS_l = capleft.get(CAP_PROP_FPS);
            cout << "Frame rate of left cam: "  << nFPS_l << endl;

            namedWindow("Left Scope Video Stream", WINDOW_NORMAL); // creat a window called "MyVideo"

            double Width_right  = capright.get(CAP_PROP_FRAME_WIDTH); // get the width of frames of the video
            double Height_right = capleft.get(CAP_PROP_FRAME_HEIGHT); // get the height of frames of the video
            cout << "Frame size of right cam: " << Width_right << " x " << Height_right << endl;

            capright.set(CAP_PROP_FPS,10);
            int nFPS_r = capright.get(CAP_PROP_FPS);
            cout << "Frame rate of right cam::" << nFPS_r << endl;
            namedWindow("Right Scope Video Stream", WINDOW_NORMAL);
        }
        ros::NodeHandle nh;
        image_transport::ImageTransport it_left(nh);
        image_transport::Publisher publ = it_left.advertise("cameras/left_hand_camera/image", 100);
        image_transport::ImageTransport it_right(nh);
        image_transport::Publisher pubr = it_right.advertise("cameras/right_hand_camera/image", 100);

        Mat frame_left;
        Mat frame_right;

        while (1)
        {
            bool leftSuccess = capleft.read(frame_left);   // read a new frame from the video
            bool rightSuccess = capright.read(frame_right);
            if (!leftSuccess || !rightSuccess) // if not success, break loop
            {
                cout <<  "ERROR: Cannot read from scopes!" << endl;
                break;
            }

            imshow("Left Scope Video Stream", frame_left);
            imshow("Right Scope Video Stream", frame_right);

//            cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
//            frame_msg = frame;
            sensor_msgs::ImagePtr msg_l = cv_bridge::CvImage( std_msgs::Header(), "bgr8", frame_left).toImageMsg();
            sensor_msgs::ImagePtr msg_r = cv_bridge::CvImage( std_msgs::Header(), "bgr8", frame_right).toImageMsg();

            if (waitKey(1) == 27)
            {
                cout <<  "ESC is pressed by user" << endl;
                break;
            }

            publ.publish(msg_l);
            pubr.publish(msg_r);
            waitKey(100); // for sync
//            ROS_INFO("%s", image_msg.data.c_str());
//            exit_key_press = cvWaitKey(1);
        }

        capleft.release();
        capright.release();
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
