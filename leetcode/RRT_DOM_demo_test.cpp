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
    Point2f start = Point2f(100, 100), end = Point2f(200, 200);
    RRT_Planner rrtPlanner1(start, end);

    cout << "Constructer1: " << rrtPlanner1.search_graph_->pos << '\n';
    //rrtPlanner1.Plan();
}