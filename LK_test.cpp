#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    const string about = 
        "This sample demonstrates Lucas-Kanade Optical Flow calculation.\n"
        "The example file can be downloaded from:\n"
        "  https://www.bogotobogo.com/python/OpenCV_Python/images/mean_shift_tracking/slow_traffic_small.mp4";
    const string keys =
        "{ h help |      | print this help message }"
        "{ @image |<none>| path to image file }";
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }  
    string filename = parser.get<string>("@image");
    if (!parser.check())
    {
        parser.printErrors();
        return 0;
    }      

    VideoCapture capture(1);
    if (!capture.isOpened())
    {
        cerr << "Unable to open the webcam!" << endl;
        return 0;
    }
    // capture.set(CAP_PROP_FPS, 30); // The default rate is also 30.

    vector<Scalar> colors;
    RNG rng;
    for (int i = 0; i < 10; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(Scalar(r, g, b));
    }

    Mat old_frame, old_gray;
    vector<Point2f> p0, p1;

    capture >> old_frame;
    cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    //goodFeaturesToTrack(old_gray, p0, 10, 0.3, 7, Mat(), 7, false, 0.04);
    p0.push_back(Point2f(300, 150));
    cout << p0[0] << endl;

    Mat mask = Mat::zeros(old_frame.size(), old_frame.type());

    while (true)
    {
        Mat frame, frame_gray;
        capture >> frame;
        if (frame.empty())
            break;
        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

        vector<uchar> status;
        vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(15, 15), 2, criteria);

        vector<Point2f> good_new;
        for (uint i = 0; i < p0.size(); i++)
        {
            if (status[i] == 1)
            {
                good_new.push_back(p1[i]);
                line(mask, p1[i], p0[i], colors[i], 2);
                circle(frame, p1[i], 5, colors[i], -1);
            }
        } 

        Mat img;
        add(frame, mask, img);
        
        imshow("Frame", img);
        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;

        old_gray = frame_gray.clone();
        //p0 = good_new;
        p0 = status[0] == 1 ? p1 : p0;
        cout << p0[0] << endl;
    }

}