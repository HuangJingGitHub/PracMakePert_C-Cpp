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


int main(int argc, char** argv) {
    Mat backImg(Size(640, 480), CV_64FC3, Scalar(255, 255, 255));
    Point2f start = Point2f(100, 100), end = Point2f(300, 300);
    RRT_Planner rrtPlanner1(start, end);

    //imshow("RRT path planning", backImg);
    //waitKey(0);
    rrtPlanner1.Plan(backImg);
    vector<RRT_Node*> path = rrtPlanner1.GetPath();
    for (int i = 0; i < path.size() - 1; i++)
        line(backImg, path[i]->pos, path[i + 1]->pos, Scalar(255,0,0), 2);   

    imshow("RRT path planning", backImg);
    waitKey(0);
    return 0;
}