#ifndef OBSTACLES_INCLUDED
#define OBSTACLES_INCLUDED

#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

struct PolygonObstacle {
    bool closed;
    vector<Point2f> vertices;
    PolygonObstacle(): closed(true) {};
    PolygonObstacle(vector<Point2f> polygon_vertices, bool is_close = true): closed(is_close), 
                                                              vertices(polygon_vertices) {}
};

int Orientation(Point2f& p1, Point2f& p2, Point2f& p3) {
    float threshold = 1e-5;
    float vec1_x = p2.x - p1.x, vec1_y = p2.y - p1.y,
          vec2_x = p3.x - p1.x, vec2_y = p3.y - p1.y;
    float cross_product = vec1_x * vec2_y - vec1_y * vec2_x;
    if (cross_product > threshold)  // > 0
        return 1;   // From segment p1-p2 to p1-p3, counterclockwise direction, CCW
    else if (cross_product < -threshold) // < 0
        return 2;    // clockwise direction
    else
        return 0;  
}

// check if point q is on the segment p1-p2
bool OnSegment(Point2f& p1, Point2f& p2, Point2f& q) {  
    if (q.x <= max(p1.x, p2.x) && q.x >= min(p1.x, p2.x)
        && q.y <= max(p1.y, p2.y) && q.y >= min(p1.y, p2.y))
            return true;
    return false;
}

bool SegmentIntersection(Point2f& p1, Point2f& p2, Point2f& q1, Point2f& q2) {
    int ori1 = Orientation(p1, p2, q1),  // segments: p1-p2, q1-q2
        ori2 = Orientation(p1, p2, q2),
        ori3 = Orientation(q1, q2, p1),
        ori4 = Orientation(q1, q2, p2);
    
    if (ori1 != ori2 && ori3 != ori4)
        return true;
    else if (ori1 == 0 && OnSegment(p1, p2, q1))
        return true;
    else if (ori2 == 0 && OnSegment(p1, p2, q2))
        return true;
    else if (ori3 == 0 && OnSegment(q1, q2, p1))
        return true;
    else if (ori4 == 0 && OnSegment(q1, q2, p2))
        return true;
    else
        return false;
}

bool ObstacleFree(PolygonObstacle& obs, Point2f p1, Point2f p2) {
    if (obs.vertices.size() <= 1)
        return true;
    for (int i = 0; i < obs.vertices.size() - 1; i++) {
        if (SegmentIntersection(obs.vertices[i], obs.vertices[i + 1], p1, p2))
            return false;
    }
    if (obs.closed)
        if (SegmentIntersection(obs.vertices[0], obs.vertices.back(), p1, p2))
            return false;
    return true;
}

float MinDistanceToObstacle(PolygonObstacle& obs, Point2f testPt) {
    float res = FLT_MAX, cur_distance;
    int step_num = 50;
    Point2f start_vertex, end_vertex, cur_interpolation_pos, side_vec;
    for (int i = 0; i < obs.vertices.size() - 1; i++) {
        start_vertex = obs.vertices[i];
        end_vertex = obs.vertices[i + 1];
        side_vec = end_vertex - start_vertex;
        for (int i = 0; i <= step_num; i++) {
            cur_interpolation_pos = start_vertex + i / step_num * side_vec;
            cur_distance = norm((testPt - cur_interpolation_pos));
            if (cur_distance < res)
                res = cur_distance;
        }
    }
    return res;
}

float MinDistanceToObstaclesVec(vector<PolygonObstacle>& obstacles, Point2f testPt) {
    float res = FLT_MAX, distance_to_obs;
    for (PolygonObstacle obs : obstacles) {
        distance_to_obs = MinDistanceToObstacle(obs, testPt);
        if (distance_to_obs < res)
            res = distance_to_obs;
    }
    return res;
}
#endif