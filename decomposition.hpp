#ifndef DECOMPOSITION_INCLUDED
#define DECOMPOSITION_INCLUDED

#include <unordered_map>
#include <opencv2/imgproc/imgproc.hpp>
#include "obstacles.hpp"

int InNegativeHalfPlane(Point2f& pt) {
    return int(pt.y < 0 || (abs(pt.y) < 1e-6 && pt.x < 0));
}

float TriplePtCross(Point2f& origin_pt, Point2f& p1, Point2f& p2) {
    return (p1 - origin_pt).cross(p2 - origin_pt);
}

string PointToString(Point2f& pt) {
    string str_x = to_string(pt.x), str_y = to_string(pt.y);
    str_x = str_x.substr(0, str_x.find(".") + 3);
    str_y = str_y.substr(0, str_y.find(".") + 3);
    return str_x + "+" + str_y;
}

vector<vector<int>> FindPlannarFaces(const vector<PolygonObstacle>& obstacles, const vector<vector<int>>& passage_pairs) {
    int obs_num = obstacles.size();
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);
    vector<vector<int>> adjacency_list(obs_num);
    for (const vector<int>& passage_pair : passage_pairs) {
        adjacency_list[passage_pair[0]].push_back(passage_pair[1]);
        adjacency_list[passage_pair[1]].push_back(passage_pair[0]);
    }
    
    std::vector<std::vector<char>> used(obs_num);
    for (int i = 0; i < obs_num; i++) {
        used[i].resize(adjacency_list[i].size());
        used[i].assign(adjacency_list[i].size(), 0);
        auto compare = [&](int l, int r) {
            Point2f pl = obs_centroids[l] - obs_centroids[i];
            Point2f pr = obs_centroids[r] - obs_centroids[i];
            if (InNegativeHalfPlane(pl) != InNegativeHalfPlane(pr))
                return InNegativeHalfPlane(pl) < InNegativeHalfPlane(pr);
            return pl.cross(pr) > 0;
        };
        std::sort(adjacency_list[i].begin(), adjacency_list[i].end(), compare);
    }

    vector<vector<int>> faces;
    for (int i = 0; i < obs_num; i++) {
        for (int edge_id = 0; edge_id < adjacency_list[i].size(); edge_id++) {
            if (used[i][edge_id]) {
                continue;
            }
            vector<int> face;
            int v = i;
            int e = edge_id;
            while (used[v][e] == false) {
                used[v][e] = true;
                face.push_back(v);
                int u = adjacency_list[v][e];
                int e1 = std::lower_bound(adjacency_list[u].begin(), adjacency_list[u].end(), v, [&](int l, int r) {
                    Point2f pl = obs_centroids[l] - obs_centroids[u];
                    Point2f pr = obs_centroids[r] - obs_centroids[u];
                    if (InNegativeHalfPlane(pl) != InNegativeHalfPlane(pr))
                        return InNegativeHalfPlane(pl) < InNegativeHalfPlane(pr);
                    return pl.cross(pr) > 0;
                }) - adjacency_list[u].begin() + 1;
                if (e1 == adjacency_list[u].size()) {
                    e1 = 0;
                }
                v = u;
                e = e1;
            }

            std::reverse(face.begin(), face.end());
            int sign = 0;
            for (int j = 0; j < face.size(); j++) {
                int j1 = (j + 1) % face.size();
                int j2 = (j + 2) % face.size();
                float val = TriplePtCross(obs_centroids[face[j]], obs_centroids[face[j1]], obs_centroids[face[j2]]);
                if (val > 0) {
                    sign = 1;
                    break;
                } else if (val < 0) {
                    sign = -1;
                    break;
                }
            }
            if (sign <= 0) {
                faces.insert(faces.begin(), face);
            } else {
                faces.emplace_back(face);
            }
        }
    }
    return faces;
}

vector<vector<int>> DelaunayTriangulationObstables(const vector<PolygonObstacle>& obs_vec, bool contain_env_walls = true, 
                                                    Size2f rect_size = Size2f(1000, 1000)) {
    vector<vector<int>> adjacency_list(obs_vec.size());
    vector<set<int>> adjacency_set(obs_vec.size());
    vector<Point2f> obstacle_centroids_vec = GetObstaclesCentroids(obs_vec);
    unordered_map<string, int> centroid_obs_map;
    for (int i = 0; i < obstacle_centroids_vec.size(); i++)
        centroid_obs_map[PointToString(obstacle_centroids_vec[i])] = i;

    cv::Rect2f bounding_box(0, 0, rect_size.width, rect_size.height);
    cv::Subdiv2D subdiv(bounding_box);
    
    // Environment boundaries, if any, are not included in trigulation.
    if (contain_env_walls == true) {
        for (int i = 4; i < obstacle_centroids_vec.size(); i++)
            subdiv.insert(obstacle_centroids_vec[i]);
    }
    else {
        for (Point2f& centroid : obstacle_centroids_vec)
            subdiv.insert(centroid);
    }

    vector<Vec6f> triangle_list;
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

    for (int i = 0; i < obs_vec.size(); i++) {
        adjacency_list[i] = vector<int>(adjacency_set[i].begin(), adjacency_set[i].end());
    }
    return adjacency_list;
}

pair<vector<vector<int>>, vector<vector<Point2f>>> PassageCheckInDelaunayGraph(const vector<PolygonObstacle>& obstacles) {
    vector<vector<int>> res_passage_pair;
    vector<vector<Point2f>> res_passage_pts;
    vector<vector<int>> adjacency_list = DelaunayTriangulationObstables(obstacles);

    int start_idx = 4; 
    for (int i = start_idx; i < obstacles.size(); i++) {
        // obstacle within geodesic distance (gd) one
        for (int j : adjacency_list[i]) {
            if (j < i)
                continue;

            vector<Point2f> cur_passage_segment_pts = GetPassageSegmentPts(obstacles[i], obstacles[j]);
            float cur_passage_length = cv::norm(cur_passage_segment_pts[0] - cur_passage_segment_pts[1]);

            set<int> obs_gd_one(adjacency_list[i].begin(), adjacency_list[i].end());
            for (int k : adjacency_list[j])
                obs_gd_one.insert(k);

            bool is_passage_valid = true;
            for (auto it = obs_gd_one.begin(); it != obs_gd_one.end(); it++) {
                int k = *it;
                if (k == i || k == j)
                    continue;
                    
                if (ObstacleFree(obstacles[k], cur_passage_segment_pts[0], cur_passage_segment_pts[1]) == false) {
                    is_passage_valid = false;
                    break;
                }
                Point2f cur_passage_center = (cur_passage_segment_pts[0] + cur_passage_segment_pts[1]) / 2;
                float cur_obs_passage_center_dist = MinDistanceToObstacle(obstacles[k], cur_passage_center);
                if (cur_obs_passage_center_dist <= cur_passage_length / 2) {
                    is_passage_valid = false;
                    break;
                }
            }
            if (is_passage_valid == true) {
                res_passage_pair.push_back({i, j});
                res_passage_pts.push_back(cur_passage_segment_pts);
            }
        }
    }
    return make_pair(res_passage_pair, res_passage_pts);
}

#endif