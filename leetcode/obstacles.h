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

float normSqr(Point2f pt) {
    return pt.x * pt.x + pt.y * pt.y;
}

struct PolygonObstacle {
    bool closed;
    vector<Point2f> vertices;
    Point2f min_distance_pt;
    PolygonObstacle(): closed(true) {};
    PolygonObstacle(vector<Point2f> polygon_vertices, bool is_close = true): closed(is_close), 
                                        vertices(polygon_vertices) {}
    PolygonObstacle(vector<Point> polygon_vertices, bool is_close = true) {
        closed = is_close;
        vertices = vector<Point2f>(polygon_vertices.size(), Point2f(0, 0));
        for (int i = 0; i < polygon_vertices.size(); i++) {
            vertices[i].x = (float) polygon_vertices[i].x;
            vertices[i].y = (float) polygon_vertices[i].y;
        }
            
    }                                                              
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

// check if point q is on the segment p1-p2 when the three points are colinear
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

bool obstacleFreeVec(vector<PolygonObstacle>& obs, Point2f p1, Point2f p2) {
    for (PolygonObstacle& obstacle : obs)
        if (!ObstacleFree(obstacle, p1, p2))
            return false;
    return true;
}

Point2f GetClosestIntersectionPt(PolygonObstacle& obs, Point2f p1, Point2f p2, Point2f testPt) {
    Point2f res = Point2f(0, 0);
    if (obs.vertices.size() <= 1) {
        cout << "Obstacle vertex number less than 1.\n";
        return res;
    }

    if (obs.closed)
        obs.vertices.push_back(obs.vertices.front());

    float min_distance_sqr = FLT_MAX;
    for (int i = 0; i < obs.vertices.size() - 1; i++) {
        if (SegmentIntersection(obs.vertices[i], obs.vertices[i + 1], p1, p2)) {
            float kp = (p2.y - p1.y) / (p2.x - p1.x),
                  kq = (obs.vertices[i + 1].y - obs.vertices[i].y) / 
                        (obs.vertices[i + 1].x - obs.vertices[i].x),
                  x_p1 = p1.x, y_p1 = p1.y,
                  x_q1 = obs.vertices[i].x, y_q1 = obs.vertices[i].y;
            float x = (y_p1 - y_q1 - kp * x_p1 + kq * x_q1) / (kq - kp),
                  y = y_p1 + kp * (x - x_p1);
            Point2f cur_intersection_pt = Point2f(x, y);
            std::cout << "test: " << cur_intersection_pt << '\n';
            if (normSqr(cur_intersection_pt - testPt) < min_distance_sqr) {
                res = cur_intersection_pt;
                min_distance_sqr = normSqr(cur_intersection_pt - testPt);
            } 
        }
    }
    
    if (obs.closed)
        obs.vertices.pop_back();
    return res;
} 


float MinDistanceToObstacle(PolygonObstacle& obs, Point2f testPt) {
    float res = FLT_MAX, cur_distance;
    int step_num = 50;
    Point2f start_vertex, end_vertex, cur_interpolation_pos, side_vec;
    
    if (obs.closed)
        obs.vertices.push_back(obs.vertices.front());
    for (int i = 0; i < obs.vertices.size() - 1; i++) {
        start_vertex = obs.vertices[i];
        end_vertex = obs.vertices[i + 1];
        side_vec = end_vertex - start_vertex;
        for (int i = 0; i <= step_num; i++) {
            cur_interpolation_pos = start_vertex + side_vec * i / step_num;
            cur_distance = normSqr(testPt - cur_interpolation_pos);
            if (cur_distance < res) {
                res = cur_distance;
                obs.min_distance_pt = cur_interpolation_pos;
            }
        }
    }
    
    if (obs.closed) 
        obs.vertices.pop_back();
    return sqrt(res);
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