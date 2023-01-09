#include "path_smoothing.h"

int main(int argc, char** argv) {
    Mat backImg(Size(640, 480), CV_64FC3, Scalar(255, 255, 255));
    Point2f start = Point2f(100, 100), end = Point2f(500, 100);
    vector<Point2f> vertices1{Point2f(310, 0), Point2f(330, 0), Point2f(330, 200), Point2f(310, 200)},
                    vertices2{Point2f(310, 280), Point2f(330, 280), Point2f(330, 480), Point2f(310, 480)},
                    vertices3{Point2f(500, 280), Point2f(520, 280), Point2f(520, 480), Point2f(500, 480)};
    PolygonObstacle obs1(vertices1), obs2(vertices2), obs3(vertices3);
    vector<PolygonObstacle> obstacles{obs1, obs2, obs3};

    RRTStarPlanner RRTStar_planer(start, end, obstacles, 20, 20, 10, Size2f(640, 480));
    bool planned = RRTStar_planer.Plan(backImg);

    vector<RRTStarNode*> path = RRTStar_planer.GetPath(), sparse_path;
    vector<Point2f> smooth_path;

    if (planned) {
        for (int i = 0; i < path.size(); i += 2)
            sparse_path.push_back(path[i]);
        if (sparse_path.back() != path.back())
            sparse_path.push_back(path.back());
        smooth_path = QuadraticBSplineSmoothing(sparse_path);
    }

    // for (int i = 0; i < int(path.size() - 1); i++)
    //     line(backImg, path[i]->pos, path[i + 1]->pos, Scalar(255,0,0), 2);   
    for (int i = 0; i < int(smooth_path.size() - 1); i++)
        line(backImg, smooth_path[i], smooth_path[i + 1], Scalar(255, 0, 0), 2);

    rectangle(backImg, Point(310, 0), Point(330, 200), Scalar(0, 0, 0), 2);
    rectangle(backImg, Point(310, 280), Point(330, 480), Scalar(0, 0, 0), 2);
    rectangle(backImg, Point(500, 280), Point(520, 480), Scalar(0, 0, 0), 2);
    // namedWindow("RRT* path planning", WINDOW_AUTOSIZE);
    imshow("RRT* path planning", backImg);
    waitKey(0);
    return 0;
}