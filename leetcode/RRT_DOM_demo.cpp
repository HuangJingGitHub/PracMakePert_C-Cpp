#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "RRT_DOM.h"
#include <iostream>
#include <string>
#include <ctype.h>
using namespace cv;
using namespace std;


static void help() {
    // print a welcome message, and the OpenCV version
    cout << "\nThis is a demo of RRT algorithm on image space,\n"
            "Using OpenCV version " << CV_VERSION << endl;
    cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
    cout << "\nHot keys: \n"
            "\tESC - quit the program\n"
            "\tc - delete all the points\n"
            << endl;
}


Point2f point;
bool addRemovePt = false;
bool saveImg = false;
static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ ) {
    if( event == EVENT_LBUTTONDOWN ) {
        point = Point2f((float)x, (float)y);
        addRemovePt = true;
    }
    else if (event == EVENT_LBUTTONDBLCLK) {
        saveImg = true;
    }
}


int main( int argc, char** argv) {
    VideoCapture cap;
    help();
    cv::CommandLineParser parser(argc, argv, "{@input|0|}");
    string input = parser.get<string>("@input");
    if( input.size() == 1 && isdigit(input[0]) )
        cap.open(input[0] - '0');
    else
        cap.open(input);
    if( !cap.isOpened() ) {
        cout << "Could not initialize capturing...\n";
        return 0;
    }
    namedWindow( "RRT Demo", 1 );
    setMouseCallback( "RRT Demo", onMouse, 0 );
    
    Mat frame;
    bool planned = false;
    vector<Point2f> points;
    // RRT_Planner rrtPlanner;
    vector<RRTNode*> path;    
    for(;;) {
        cap >> frame;
        if( frame.empty() )
            break;

        if( addRemovePt && points.size() < 2 ) {
            points.push_back(point);
            addRemovePt = false;
        }
        if (points.size() == 2 && !planned) {
            RRTPlanner rrtPlanner (points[0], points[1], 5, 5);
            
            cout << "Planning...\n";
            planned = rrtPlanner.Plan(frame);
            path = rrtPlanner.GetPath();

        }

        if (points.size() == 2) {
            circle(frame, points[0], 3, Scalar(0,255,0), -1, 8);
            circle(frame, points[1], 3, Scalar(0,255,0), -1, 8);
        }

        if (planned) {
            for (int i = 0; i < path.size() - 1; i++)
                line(frame, path[i]->pos, path[i + 1]->pos, Scalar(0,0,255), 2);
        }

        imshow("RRT Demo", frame);
        char c = (char)waitKey(10);
        if( c == 27 )
            break;
        switch( c ) {
        case 'c':
            points.clear();
            break;
        }
    }
    return 0;
}