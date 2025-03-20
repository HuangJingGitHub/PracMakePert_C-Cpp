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
        vector<Point2f> pts = SVIntersection(obstacles[0].Get2dObstacle(), obstacles[1].Get2dObstacle()).back();
        vector<float> heights = {obstacles[0].height, obstacles[1].height};
        sort(heights.begin(), heights.end());
        
        Passages3d res;
        res.pairs = vector<vector<int>>(1, pair);
        res.pts = vector<vector<Point2f>>(1, pts);
        res.heights = vector<vector<float>>(1, heights);
        return res;
    }   

    vector<int> sort_obs_indices(obstacles.size(), 0);
    for (int i = 0; i < sort_obs_indices.size(); i++)
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
    
    vector<vector<int>> res_psg_pairs;
    vector<vector<Point2f>> res_psg_pts;
    vector<vector<float>> res_psg_heights;
    vector<bool> res_psg_processed;

    // Skip four environment walls. Walls have the maximum heihgt. Their indices are thus in the front.
    // Initialization
    Rect2f bounding_box(0, 0, 10000, 10000);
    Subdiv2D subdiv(bounding_box);
    int highest_idx = sort_obs_indices[4],
        next_highest_idx = sort_obs_indices[5];
    subdiv.insert(obs_centroids[highest_idx]);
    subdiv.insert(obs_centroids[next_highest_idx]);
    res_psg_pairs.push_back({min(highest_idx, next_highest_idx), max(highest_idx, next_highest_idx)});
    res_psg_pts.push_back(SVIntersection(obstacles[highest_idx].Get2dObstacle(), 
                                        obstacles[next_highest_idx].Get2dObstacle()).back());
    res_psg_heights.push_back({0, obstacles[next_highest_idx].height});
    res_psg_processed.push_back(false);

    vector<set<int>> pre_adjacency_set(obstacles.size());
    for (int sort_idx = 6; sort_idx < sort_obs_indices.size(); sort_idx++) {
        int obs_idx = sort_obs_indices[sort_idx];
        
        vector<Vec6f> triangle_list;
        vector<set<int>> adjacency_set(obstacles.size());
        subdiv.insert(obs_centroids[obs_idx]);
        subdiv.getTriangleList(triangle_list);
        for (Vec6f& triangle : triangle_list) {
            Point2f vertex_1(triangle[0], triangle[1]), 
                    vertex_2(triangle[2], triangle[3]), 
                    vertex_3(triangle[4], triangle[5]);
            string key_str_1 = PointToString(vertex_1),
                key_str_2 = PointToString(vertex_2),
                key_str_3 = PointToString(vertex_3);
            int obs_idx_1 = centroid_obs_map[key_str_1],
                obs_idx_2 = centroid_obs_map[key_str_2],
                obs_idx_3 = centroid_obs_map[key_str_3];
            adjacency_set[obs_idx_1].insert(obs_idx_2);
            adjacency_set[obs_idx_1].insert(obs_idx_3);
            adjacency_set[obs_idx_2].insert(obs_idx_1);
            adjacency_set[obs_idx_2].insert(obs_idx_3);
            adjacency_set[obs_idx_3].insert(obs_idx_1);
            adjacency_set[obs_idx_3].insert(obs_idx_2); 
        }

        set<int> obs_gd_two = adjacency_set[obs_idx];
        for (int obs_gd_1 : adjacency_set[obs_idx])
           for (int obs_gd_2 : adjacency_set[obs_gd_1]) 
                obs_gd_two.insert(obs_gd_2);

        // Check previously detected passages.
        for (int psg_idx = 0; psg_idx < res_psg_pairs.size(); psg_idx++) {
            if (res_psg_processed[psg_idx])
                continue;

            int idx_1 = res_psg_pairs[psg_idx][0], idx_2 = res_psg_pairs[psg_idx][1];
            // NO NEED to check if previous passages still remain as edges, a passage is knocked out 
            // only if it fails the detection condition. In the dynamic Delaunay graph building process, 
            // it is possible that an edge is first established, eliminated, and possibly rebuilt again, 
            // so on and so furth. Edge existance is not a criterion of passage validity.
            /*if (adjacency_set[idx_1].count(idx_2) == 0) {
                res_psg_heights[psg_idx][0] = obstacles[obs_idx].height;
                res_psg_processed[psg_idx] = true;
                continue;
            }*/

            if (obs_gd_two.count(idx_1) || obs_gd_two.count(idx_2)) {
                vector<vector<Point2f>> psg_key_pts = SVIntersection(obstacles[idx_1].Get2dObstacle(), 
                                                                    obstacles[idx_2].Get2dObstacle());
                if (!ObstacleFree(obstacles[obs_idx].Get2dObstacle(), psg_key_pts[0][0], psg_key_pts[0][1])
                    || !ObstacleFree(obstacles[obs_idx].Get2dObstacle(), psg_key_pts[1][0], psg_key_pts[1][1])) {
                    res_psg_heights[psg_idx][0] = obstacles[obs_idx].height;
                    res_psg_processed[psg_idx] = true;
                    continue;
                }
                
                vector<Point2f> psg_segment_pts = psg_key_pts.back();
                float psg_length = cv::norm(psg_segment_pts[0] - psg_segment_pts[1]);   
                Point2f psg_center = (psg_segment_pts[0] + psg_segment_pts[1]) / 2;
                float obs_psg_center_dist = MinDistanceToObstacle(obstacles[obs_idx].Get2dObstacle(), psg_center);
                if (obs_psg_center_dist <= psg_length / 2) {
                    res_psg_heights[psg_idx][0] = obstacles[obs_idx].height;
                    res_psg_processed[psg_idx] = true;
                    continue;
                }                
            }
        }

        for (int j : obs_gd_two) {
            if (j == obs_idx)
                continue;

            int idx_1 = min(j, obs_idx), idx_2 = max(j, obs_idx);
            vector<vector<Point2f>> psg_key_pts = SVIntersection(obstacles[idx_1].Get2dObstacle(), obstacles[idx_2].Get2dObstacle());
            vector<Point2f> psg_segment_pts = psg_key_pts.back();
            float psg_length = cv::norm(psg_segment_pts[0] - psg_segment_pts[1]);
            // Obstacles within geodesic distance (gd) two of two obstacles
            set<int> extended_obs_gd_two = obs_gd_two;
            for (int k : adjacency_set[j]) {
                extended_obs_gd_two.insert(k);
                for (int l : adjacency_set[k])
                    extended_obs_gd_two.insert(l);
            }

            bool is_psg_valid = true;
            for (int k : extended_obs_gd_two) {
                if (k == obs_idx || k == j)
                    continue;
                
                if (!ObstacleFree(obstacles[k].Get2dObstacle(), psg_key_pts[0][0], psg_key_pts[0][1])
                    || !ObstacleFree(obstacles[k].Get2dObstacle(), psg_key_pts[1][0], psg_key_pts[1][1])) {
                    is_psg_valid = false;
                    break;
                }
                Point2f psg_center = (psg_segment_pts[0] + psg_segment_pts[1]) / 2;
                float obs_psg_center_dist = MinDistanceToObstacle(obstacles[k].Get2dObstacle(), psg_center);
                if (obs_psg_center_dist <= psg_length / 2) {
                    is_psg_valid = false;                    
                    break;
                }
            }
            if (is_psg_valid) {
                res_psg_pairs.push_back({idx_1, idx_2});
                res_psg_pts.push_back(psg_segment_pts);
                res_psg_heights.push_back({0, obstacles[obs_idx].height});
                res_psg_processed.push_back(false);
            }
        }      
        pre_adjacency_set = adjacency_set;
    }

    return Passages3d(res_psg_pairs, res_psg_pts, res_psg_heights);
}
#endif