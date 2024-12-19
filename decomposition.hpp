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

vector<vector<int>> DelaunayTriangulationObstables(const vector<PolygonObstacle>& obs_vec, 
                                                    bool contain_env_walls = false, 
                                                    Size2f rect_size = Size2f(3000, 3000)) {
    vector<vector<int>> adjacency_list(obs_vec.size());
    vector<set<int>> adjacency_set(obs_vec.size());
    vector<Point2f> obs_centroids_vec = GetObstaclesCentroids(obs_vec);
    obs_centroids_vec[0].y = 0;
    obs_centroids_vec[2].x = 0;

    unordered_map<string, int> centroid_obs_map;
    for (int i = 0; i < obs_centroids_vec.size(); i++)
        centroid_obs_map[PointToString(obs_centroids_vec[i])] = i;

    Rect2f bounding_box(0, 0, rect_size.width, rect_size.height);
    Subdiv2D subdiv(bounding_box);
    
    // Environment boundaries, if any, are not included in trigulation.
    if (contain_env_walls == false) {
        for (int i = 4; i < obs_centroids_vec.size(); i++)
            subdiv.insert(obs_centroids_vec[i]);
    }
    else {
        for (Point2f& centroid : obs_centroids_vec)
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
    vector<vector<int>> res_psg_pair;
    vector<vector<Point2f>> res_psg_pts;
    vector<vector<int>> adjacency_list = DelaunayTriangulationObstables(obstacles, true);

    // Explicitly do not process environment walls here.
    int start_idx = 4; 
    for (int i = start_idx; i < obstacles.size(); i++) {
        // obstacle within geodesic distance (gd) two
        set<int> neighbor_obs_gd_two(adjacency_list[i].begin(), adjacency_list[i].end());
        for (int neighbor_gd_1 : adjacency_list[i])
           for (int neighbor_gd_2 : adjacency_list[neighbor_gd_1]) 
                neighbor_obs_gd_two.insert(neighbor_gd_2);

        for (int j : neighbor_obs_gd_two) {
            // Do not check boundary pairs
            if (j <= i)
                continue;

            vector<vector<Point2f>> psg_key_pts = SVIntersection(obstacles[i], obstacles[j]);
            vector<Point2f> psg_segment_pts = psg_key_pts.back();
            // vector<Point2f> psg_segment_pts = GetPassageSegmentPts(obstacles[i], obstacles[j]);
            float psg_length = cv::norm(psg_segment_pts[0] - psg_segment_pts[1]);
            // obstacle within geodesic distance (gd) two of two obstacles
            set<int> obs_gd_two = neighbor_obs_gd_two;
            for (int k : adjacency_list[j])
                obs_gd_two.insert(k);
            for (int k : adjacency_list[j])
                for (int l : adjacency_list[k])
                    obs_gd_two.insert(l);

            bool is_psg_valid = true;
            for (auto it = obs_gd_two.begin(); it != obs_gd_two.end(); it++) {
                int k = *it;
                if (k == i || k == j)
                    continue;
                
                // Assumption: if an obstacle collides with the passage region,   
                // it must collide with  one of the passage region boundaries.
                if (ObstacleFree(obstacles[k], psg_key_pts[0][0], psg_key_pts[0][1]) == false
                    || ObstacleFree(obstacles[k], psg_key_pts[1][0], psg_key_pts[1][1]) == false) {
                    is_psg_valid = false;
                    break;
                }
                Point2f psg_center = (psg_segment_pts[0] + psg_segment_pts[1]) / 2;
                float obs_psg_center_dist = MinDistanceToObstacle(obstacles[k], psg_center);
                if (obs_psg_center_dist <= psg_length / 2) {
                    is_psg_valid = false;                    
                    break;
                }
            }
            if (is_psg_valid == true) {
                res_psg_pair.push_back({i, j});
                res_psg_pts.push_back(psg_segment_pts);
            }
        }
    }
    return make_pair(res_psg_pair, res_psg_pts);
}

pair<vector<vector<int>>, vector<vector<Point2f>>> PassageCheckDelaunayGraphWithWalls(const vector<PolygonObstacle>& obstacles) {
    pair<vector<vector<int>>, vector<vector<Point2f>>> res_env_walls = ExtendedVisibilityCheckForWalls(obstacles);
    pair<vector<vector<int>>, vector<vector<Point2f>>> res_obs = PassageCheckInDelaunayGraph(obstacles);
    res_env_walls.first.insert(res_env_walls.first.end(), res_obs.first.begin(), res_obs.first.end());
    res_env_walls.second.insert(res_env_walls.second.end(), res_obs.second.begin(), res_obs.second.end());
    return res_env_walls;
}

vector<vector<int>> FindPlannarFaces(const vector<PolygonObstacle>& obstacles, const vector<vector<int>>& passage_pairs) {
    int obs_num = obstacles.size();
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);
    // Expand centroids of environment walls so that their relative  
    // directions w.r.t. one another are correctly computed.
    obs_centroids[0].y -= 1000;
    obs_centroids[1].y += 1000;
    obs_centroids[2].x -= 1000;
    obs_centroids[3].x += 1000;

    vector<vector<int>> adjacency_list(obs_num);
    for (const vector<int>& passage_pair : passage_pairs) {
        adjacency_list[passage_pair[0]].push_back(passage_pair[1]);
        adjacency_list[passage_pair[1]].push_back(passage_pair[0]);
    }
    
    std::vector<std::vector<char>> used(obs_num);
    for (int i = 0; i < obs_num; i++) {
        used[i].resize(adjacency_list[i].size());
        used[i].assign(adjacency_list[i].size(), 0);
        // Sort points by axis angles. Two cases for transitivity: 1) on the same half plane, 
        // 2) on different half planes. Solely using cross product is insufficient in comparer.
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
            if (used[i][edge_id] == true) {
                continue;
            }
            vector<int> face;
            int v = i;
            int e = edge_id;
            while (used[v][e] == false) {
                used[v][e] = true;
                face.push_back(v);
                int u = adjacency_list[v][e];
                int e_1 = std::lower_bound(adjacency_list[u].begin(), adjacency_list[u].end(), v, [&](int l, int r) {
                    Point2f pl = obs_centroids[l] - obs_centroids[u];
                    Point2f pr = obs_centroids[r] - obs_centroids[u];
                    if (InNegativeHalfPlane(pl) != InNegativeHalfPlane(pr))
                        return InNegativeHalfPlane(pl) < InNegativeHalfPlane(pr);
                    return pl.cross(pr) > 0;
                }) - adjacency_list[u].begin() + 1;
                if (e_1 == adjacency_list[u].size()) {
                    e_1 = 0;
                }
                v = u;
                e = e_1;
            }

            // std::reverse(face.begin(), face.end());
            int sign = 0;
            for (int j = 0; j < face.size(); j++) {
                int j_1 = (j + 1) % face.size();
                int j_2 = (j + 2) % face.size();
                float val = TriplePtCross(obs_centroids[face[j]], obs_centroids[face[j_1]], obs_centroids[face[j_2]]);
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

vector<vector<int>> ReportGabrielCells(const vector<PolygonObstacle>& obstacles, const vector<vector<int>>& passage_pairs) {
    vector<vector<int>> augmented_psg_pairs = passage_pairs, res;
    // Add pseudo passage between four environment walls.
    augmented_psg_pairs.push_back({0, 2});
    augmented_psg_pairs.push_back({0, 3});
    augmented_psg_pairs.push_back({1, 2});
    augmented_psg_pairs.push_back({1, 3});

    res = FindPlannarFaces(obstacles, augmented_psg_pairs);
    return res;
}

pair<vector<vector<int>>, vector<vector<vector<Point2f>>>> GetGabrielCellInfo(const vector<vector<int>>& cells, 
                                                                              const pair<vector<vector<int>>, vector<vector<Point2f>>>& psg_res) {
    // Add pseudo passage between four environment walls.
    vector<vector<int>> augmented_psg_pairs = psg_res.first;
    augmented_psg_pairs.push_back({0, 2});
    augmented_psg_pairs.push_back({0, 3});
    augmented_psg_pairs.push_back({1, 2});
    augmented_psg_pairs.push_back({1, 3});
    // Set invalid passage point positions such that no intersection with path segments is possible.
    vector<vector<Point2f>> augmented_psg_pts = psg_res.second;
    augmented_psg_pts.push_back({Point2f(0, -1), Point2f(-1, 0)});
    augmented_psg_pts.push_back({Point2f(10000, -1), Point2f(10001, 0)});
    augmented_psg_pts.push_back({Point2f(0, 10001), Point2f(-1, 10000)});
    augmented_psg_pts.push_back({Point2f(10000, 10001), Point2f(10001, 10000)});   

    unordered_map<string, vector<Point2f>> psg_obs_to_pt_map;
    int psg_num = augmented_psg_pairs.size();
    for (int i = 0; i < psg_num; i++) {
        string key_str = to_string(augmented_psg_pairs[i][0]) + "-" + to_string(augmented_psg_pairs[i][1]);
        psg_obs_to_pt_map[key_str] = augmented_psg_pts[i];
    }

    int cell_num = cells.size();
    vector<vector<vector<Point2f>>> cell_side_pts(cell_num);
    for (int i = 0; i < cell_num; i++) {
        int vertex_num = cells[i].size();
        for (int j = 0; j < vertex_num; j++) {
            int obs_idx_1 = min(cells[i][j], cells[i][(j + 1) % vertex_num]),
                obs_idx_2 = max(cells[i][j], cells[i][(j + 1) % vertex_num]);
            string key_str = to_string(obs_idx_1) + "-" + to_string(obs_idx_2);
            if (psg_obs_to_pt_map.count(key_str) == 0)
                throw std::runtime_error("Passages as cell side not existing in detection results in " + string(__func__));

            cell_side_pts[i].push_back(psg_obs_to_pt_map[key_str]);
        }
    }

    return make_pair(cells, cell_side_pts);
}

#endif