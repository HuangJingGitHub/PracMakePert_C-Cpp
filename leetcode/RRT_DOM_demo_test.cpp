#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "RRT_DOM.h"
#include "RRTStar_DOM_debug.h"
#include <iostream>
#include <string>
#include <ctype.h>
using namespace cv;
using namespace std;


int main(int argc, char** argv) {
    Mat backImg(Size(640, 480), CV_64FC3, Scalar(255, 255, 255));
    Point2f start = Point2f(100, 100), end = Point2f(400, 200);
    RRTStarPlanner rrtPlanner1(start, end, 15);
    // RRTPlanner rrtPlanner1(start, end, 15);

    //imshow("RRT path planning", backImg);
    //waitKey(0);
    rrtPlanner1.Plan(backImg);
    vector<RRTStarNode*> path = rrtPlanner1.GetPath();
    for (int i = 0; i < path.size() - 1; i++)
        line(backImg, path[i]->pos, path[i + 1]->pos, Scalar(255,0,0), 2);   

    imshow("RRT path planning", backImg);
    waitKey(0);
    return 0;
}