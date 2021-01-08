#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <string>
#include <ctype.h>
#include "RRT_DOM.h"
#include "RRTStar_DOM_debug.h"
using namespace cv;
using namespace std;


int main(int argc, char** argv) {
    Mat backImg(Size(640, 480), CV_64FC3, Scalar(255, 255, 255));
    Point2f start = Point2f(100, 100), end = Point2f(500, 200);
    vector<Point2f> vertices1{Point2f(310, 0), Point2f(330, 0), Point2f(330, 200), Point2f(310, 200)},
                    vertices2{Point2f(310, 280), Point2f(330, 280), Point2f(330, 480), Point2f(310, 480)};
    PolyObstacle obs1(vertices1), obs2(vertices2);
    vector<PolyObstacle> obstacles{obs1, obs2};

    RRTStarPlanner rrtPlanner1(start, end, obstacles, 25);
    // RRTPlanner rrtPlanner1(start, end, 15);

    // imshow("RRT path planning", backImg);
    // waitKey(0);
    rrtPlanner1.Plan(backImg);
    vector<RRTStarNode*> path = rrtPlanner1.GetPath();
    for (int i = 0; i < int(path.size() - 1); i++)
        line(backImg, path[i]->pos, path[i + 1]->pos, Scalar(255,0,0), 2);   

    rectangle(backImg, Point(310, 0), Point(330, 200), Scalar(0, 0, 0), -1);
    rectangle(backImg, Point(310, 280), Point(330, 480), Scalar(0, 0, 0), -1);
    imshow("RRT path planning", backImg);
    waitKey(0);
    return 0;
}