#ifndef DECOMPOSITION3D_INCLUDED
#define DECOMPOSITION3D_INCLUDED

#include <unordered_map>
#include <opencv2/imgproc/imgproc.hpp>
#include "obstacles_3d.hpp"
#include "kd_tree_3d.hpp"
#include "../decomposition.hpp"

Passages3d PassageCheckInDelaunayGraph3d(const vector<PolygonObstacle3d>& obstacles, bool contain_env_walls = true) {
    if (obstacles.size() < 2) {
        string msg = "Obstacle number is less than two in " + string(__func__);
        throw std::invalid_argument(msg);        
    }
    else if (obstacles.size() == 2) {
        vector<int> pair = {0, 1};
        vector<Point2f> pts = SVIntersection(obstacles[0], obstacles[1]).back();
        vector<float> heights = {obstacles[0].height, obstacles[1].height};
        sort(heights.begin(), heights.end());
        
        Passages3d res;
        res.pairs = vector<vector<int>>(1, pair);
        res.pts = vector<vector<Point2f>>(1, pts);
        res.heights = vector<vector<float>>(1, heights);
        return res;
    }   

    vector<size_t> sort_obs_indices(obstacles.size(), 0);
    for (size_t i = 0; i < sort_obs_indices.size(); i++)
        sort_obs_indices[i] = i;
    stable_sort(sort_obs_indices.begin(), sort_obs_indices.end(), 
                [&obstacles](size_t idx_1, size_t idx_2) {return obstacles[idx_1].height > obstacles[idx_2].height;});

    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);
    
    Point2f shift(1000, 1000);
    for (Point2f& centroid : obs_centroids)
        centroid += shift;

    unordered_map<string, int> centroid_obs_map;
    for (int i = 0; i < obs_centroids.size(); i++)
        centroid_obs_map[PointToString(obs_centroids[i])] = i;

    Rect2f bounding_box(0, 0, 10000, 10000);
    Subdiv2D subdiv(bounding_box);

}


#endif