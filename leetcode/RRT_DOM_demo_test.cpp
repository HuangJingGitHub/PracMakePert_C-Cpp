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

class TEST {
public:
    int* intpt;
    TEST(): intpt(nullptr) {}
    TEST(int a) {
        intpt = new int(a);
    }
    ~TEST() {
        delete intpt;
    }
};


int main(int argc, char** argv) {
    Point2f start = Point2f(100, 100), end = Point2f(200, 200);
    RRT_Planner rrtPlanner1(start, end), rrtPlanner2;
    //rrtPlanner2 = RRT_Planner(start, end);

    cout << "Constructer1: " << rrtPlanner1.search_graph_->pos << '\n';

    int int1 = 10;
    TEST test1(int1), test2;
    test2 = TEST(int1);
    cout << *(test2.intpt) << '\n';
    return 0;
}