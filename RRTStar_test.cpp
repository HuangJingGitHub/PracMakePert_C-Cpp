#include "path_processing.hpp"

int main(int argc, char** argv) {
    Mat backImg(Size(640, 480), CV_64FC3, Scalar(255, 255, 255));
    Point2f start = Point2f(100, 110), end = Point2f(540, 110);
    vector<Point2f> initial_feedback_pts{start, Point2f(120, 80)};
    int pivot_idx = 0;
    vector<Point2f> vertices1{Point2f(300, 0), Point2f(340, 0), Point2f(340, 100), Point2f(300, 100)},          
                    vertices2{Point2f(300, 320), Point2f(340, 320), Point2f(340, 480), Point2f(300, 480)},
                    vertices3{Point2f(300, 150), Point2f(340, 150), Point2f(340, 200), Point2f(300, 200)}, 
                    vertices4{Point2f(480, 280), Point2f(520, 280), Point2f(520, 480), Point2f(480, 480)};
    PolygonObstacle obs1(vertices1), obs2(vertices2), obs3(vertices3), obs4(vertices4);
    vector<PolygonObstacle> obstacles{obs1, obs2, obs3, obs4};
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);

    RRTStarPlanner RRTStar_planer(start, end, obstacles, 20, 20, 10, Size2f(640, 480));
    bool planned = RRTStar_planer.Plan(backImg, 2, true);

    vector<RRTStarNode*> path = RRTStar_planer.GetPath(), sparse_path;
    vector<Point2f> smooth_path;

    if (planned) {
        for (int i = 0; i < path.size(); i += 2)
            sparse_path.push_back(path[i]);
        if (sparse_path.back() != path.back())
            sparse_path.push_back(path.back());
        smooth_path = QuadraticBSplineSmoothing(sparse_path);
    }

    // std::cout << "Raw path node num: " << path.size() << ". Smoothed path node num: " << smooth_path.size() << "\n";
    
    vector<vector<int>> passage_passed = GetPassagesPathPasses(obstacles, path);  
    vector<vector<int>> intersection_idx; 
    vector<vector<Point2f>> passage_intersection_pts = GetPassageIntersectionsOfPathSet(obstacles, passage_passed, smooth_path, intersection_idx, initial_feedback_pts, 0);
    vector<Point2f> reposition_points = GetPivotPathRepositionPts(obstacles, passage_passed, passage_intersection_pts, pivot_idx);
    DeformPath(smooth_path, passage_intersection_pts[0], reposition_points, intersection_idx[0]);

    // Deform transferred paths---> Write a function for it
    vector<vector<Point2f>> path_set(initial_feedback_pts.size(), smooth_path);
    for (int i = 0; i < initial_feedback_pts.size(); i++) {
        if (i == pivot_idx) 
            continue;

        for (Point2f& path_node : path_set[i])
             path_node += initial_feedback_pts[i] - initial_feedback_pts[pivot_idx];
        /*Deal with the last node, descrete the segment s_ref,i to s_d,i */

        vector<vector<int>>
    }

    for (int i = 0; i < int(path.size() - 1); i++) {
        line(backImg, path[i]->pos, path[i + 1]->pos, Scalar(255,0,0), 2);         
        line(backImg, path[i]->pos + initial_feedback_pts[1] - initial_feedback_pts[0], path[i + 1]->pos + initial_feedback_pts[1] - initial_feedback_pts[0], Scalar(255, 0, 0), 2); 
    } 

    for (vector<int>& passage : passage_passed) {
        line(backImg, obs_centroids[passage[0]], obs_centroids[passage[1]], Scalar(0, 0, 255), 2);
    }
    for (auto node_passage_intersection_pts : passage_intersection_pts)
        for (auto pt : node_passage_intersection_pts)
            circle(backImg, pt, 2, Scalar(0, 0, 0), 2);

    for (int i = 0; i < smooth_path.size() - 1; i++)
        line(backImg, smooth_path[i], smooth_path[i + 1], Scalar(0,255,0), 2);
    circle(backImg, smooth_path.front(), 4, Scalar(0, 0, 0), -1);
    circle(backImg, smooth_path.back(), 4, Scalar(0, 0, 0), -1);

    rectangle(backImg, vertices1[0], vertices1[2], Scalar(0, 0, 0), 2);
    putText(backImg, "1", Point(310, 50), FONT_ITALIC, 1.2, Scalar(255, 0, 0), 2);
    rectangle(backImg, vertices2[0], vertices2[2], Scalar(0, 0, 0), 2);
    putText(backImg, "2", Point(310, 180), FONT_ITALIC, 1.2, Scalar(255, 0, 0), 2);  
    rectangle(backImg, vertices3[0], vertices3[2], Scalar(0, 0, 0), 2);
    putText(backImg, "3", Point(310, 400), FONT_ITALIC, 1.2, Scalar(255, 0, 0), 2);
    rectangle(backImg, vertices4[0], vertices4[2], Scalar(0, 0, 0), 2);
    putText(backImg, "4", Point(490, 380), FONT_ITALIC, 1.2, Scalar(255, 0, 0), 2);
    // namedWindow("RRT* path planning", WINDOW_AUTOSIZE);
    imshow("RRT* path planning", backImg);
    waitKey(0); 
    return 0;
}