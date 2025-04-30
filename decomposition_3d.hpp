#ifndef DECOMPOSITION3D_INCLUDED
#define DECOMPOSITION3D_INCLUDED

#include <unordered_map>
#include <opencv2/imgproc/imgproc.hpp>
#include "obstacles_3d.hpp"
#include "kd_tree_3d.hpp"
#include "../decomposition.hpp"

struct PolygonCell3d {
    vector<int> obs_indices;
    vector<Point2f> vertices;
    vector<vector<int>> intersect_psg_pairs;
    vector<vector<Point2f>> intersect_psg_pts;
    bool is_an_obstacle = false;
    PolygonCell3d() {}
    PolygonCell3d(vector<int> cell_obs_indices, vector<Point2f> cell_vertices): obs_indices(cell_obs_indices), vertices(cell_vertices) {}
};

Passages3d PassageCheckInDelaunayGraph3d(const vector<PolygonObstacle3d>& obstacles) {
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

    // Skip four environment walls. Walls are not processed in this function.
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

    for (int sort_idx = 6; sort_idx < sort_obs_indices.size(); sort_idx++) {
        int obs_idx = sort_obs_indices[sort_idx];
        
        // Do not retrieve edge list because bounding points will be  
        // included in the returned edge list.
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
    
        // gd: geodesic distance
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
            // it is possible that an edge is first established, eliminated, and rebuilt again, 
            // so on and so furth. Edge existance is not a criterion of passage validity.

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
    }
    return Passages3d(res_psg_pairs, res_psg_pts, res_psg_heights);
}

// Detection of 3d passages above does not take into account environment walls.
// The results thus need to be further filtered if environment boundaries are there.
void FilterPassagesWithWalls3d(Passages3d& passages, Size2f config_size = Size2f(1000, 600)) {
    set<int> invalid_indices;
    float max_width = config_size.width, max_height = config_size.height;

    for (int i = 0; i < passages.pairs.size(); i++) {
        Point2f psg_center = (passages.pts[i][0] + passages.pts[i][1]) / 2;
        float psg_length = cv::norm(passages.pts[i][0] - passages.pts[i][1]);
        float obs_psg_center_dist = min(min(psg_center.x, max_width - psg_center.x), 
                                        min(psg_center.y, max_height - psg_center.y));
        if (obs_psg_center_dist <= psg_length / 2) {
            invalid_indices.insert(i);
        }
    }

    int i = 0, j = 0;
    for (; j < passages.pairs.size();) {
        if (invalid_indices.count(j))
            j++;
        else {
            passages.pairs[i] = passages.pairs[j];
            passages.pts[i] = passages.pts[j];
            passages.heights[i] = passages.heights[j];
            i++; j++;
        }
    }
    passages.pairs.resize(i);
    passages.pts.resize(i);
    passages.heights.resize(i);
}


Passages3d PassageCheckForWalls3d(const vector<PolygonObstacle3d>& obstacles) {
    vector<vector<int>> res_psg_pairs;
    vector<vector<Point2f>> res_psg_pts;
    vector<vector<float>> res_psg_heights;

    vector<int> sort_obs_indices(obstacles.size(), 0);
    for (int i = 0; i < sort_obs_indices.size(); i++)
        sort_obs_indices[i] = i;
    stable_sort(sort_obs_indices.begin(), sort_obs_indices.end(), 
                [&obstacles](size_t idx_1, size_t idx_2) {return obstacles[idx_1].height > obstacles[idx_2].height;});
    
    for (int i = 0; i < 4; i++)
        for (int j = 4; j < obstacles.size(); j++) {
            int obs_idx_1 = i, obs_idx_2 = sort_obs_indices[j];
            vector<vector<Point2f>> psg_key_pts = SVIntersection(obstacles[obs_idx_1].Get2dObstacle(), obstacles[obs_idx_2].Get2dObstacle());
            vector<Point2f> psg_segment_pts = psg_key_pts.back();
            float psg_length = cv::norm(psg_segment_pts[0] - psg_segment_pts[1]);            

            int k = 0;
            for (; k < obstacles.size(); k++) {
                if (k == i || k == j)
                    continue;
                int obs_idx_3 = sort_obs_indices[k];

                if (!ObstacleFree(obstacles[obs_idx_3].Get2dObstacle(), psg_key_pts[0][0], psg_key_pts[0][1])
                    || !ObstacleFree(obstacles[obs_idx_3].Get2dObstacle(), psg_key_pts[1][0], psg_key_pts[1][1])) {
                    break;
                    }
                Point2f psg_center = (psg_segment_pts[0] + psg_segment_pts[1]) / 2;
                float obs_psg_center_dist = MinDistanceToObstacle(obstacles[obs_idx_3].Get2dObstacle(), psg_center);
                if (obs_psg_center_dist <= psg_length / 2) {
                    break;
                }
            }
            if (k > j) {
                res_psg_pairs.push_back({obs_idx_1, obs_idx_2});
                res_psg_pts.push_back(psg_segment_pts);
                res_psg_heights.push_back({k == obstacles.size() ? 0 : obstacles[sort_obs_indices[k]].height, obstacles[obs_idx_2].height});
            }
        }
    return Passages3d(res_psg_pairs, res_psg_pts, res_psg_heights);
}

Passages3d PassageCheckDelaunayGraphWithWalls3d(const vector<PolygonObstacle3d>& obstacles) {
    Passages3d passages = PassageCheckInDelaunayGraph3d(obstacles);
    FilterPassagesWithWalls3d(passages);
    Passages3d passages_wall = PassageCheckForWalls3d(obstacles);   
    
    passages_wall.pairs.insert(passages_wall.pairs.end(), passages.pairs.begin(), passages.pairs.end());
    passages_wall.pts.insert(passages_wall.pts.end(), passages.pts.begin(), passages.pts.end());
    passages_wall.heights.insert(passages_wall.heights.end(), passages.heights.begin(), passages.heights.end());
    return passages_wall;
}


vector<PolygonCell3d> GetCompoundGabrielCells3d(const vector<PolygonObstacle3d>& obstacles) {
    Passages3d passages_3d = PassageCheckDelaunayGraphWithWalls3d(obstacles);

    vector<PolygonObstacle> obstacles_2d = ConvertTo2dObstacles(obstacles);
    Passages passages_2d = PassageCheckDelaunayGraphWithWalls(obstacles_2d);

    vector<vector<int>> cells = ReportGabrielCells(obstacles_2d, passages_2d.pairs, true);
    vector<PolygonCell> cells_2d = GetGabrielCellsInfo(cells, passages_2d);
    return {};
}

vector<vector<int>> GetPassedCellsAndObstacles(const Point2f& start_pt, const Point2f& end_pt,
                                              const vector<PolygonCell>& cells, const vector<PolygonObstacle>& obstacles) {
    vector<int> res_cells, res_obstacles;
    unordered_map<string, vector<int>> psg_cell_idx_map = GetPassageCellMap(cells);

    Point2f direction = (end_pt - start_pt) / cv::norm(end_pt - start_pt),
            new_start_pt = start_pt + 0.5 * direction,
            new_end_pt = end_pt - 0.5 * direction;
    int start_cell_idx = LocatePtInCells(new_start_pt, cells),
        end_cell_idx = LocatePtInCells(new_end_pt, cells);
    res_cells.push_back(start_cell_idx);
    set<string> passed_psg_pairs;

    while (start_cell_idx != end_cell_idx) {
        const PolygonCell* cell = &cells[start_cell_idx];
        int obs_num = cell->obs_indices.size();
        for (int i = 0; i < obs_num; i++) {
            Point2f vertex_1 = cell->vertices[2 * i],
                    vertex_2 = cell->vertices[2 * i + 1];
            if (SegmentIntersection(vertex_1, vertex_2, new_start_pt, end_pt)) {
                string key_str = PassagePairToKeyString(cell->obs_indices[i], cell->obs_indices[(i + 1) % obs_num]);
                if (passed_psg_pairs.count(key_str) > 0)
                    continue;

                passed_psg_pairs.insert(key_str);
                for (int cell_idx : psg_cell_idx_map[key_str]) {
                    if (cell_idx != start_cell_idx) {
                        start_cell_idx = cell_idx;
                        res_cells.push_back(start_cell_idx);
                        break;
                    }
                }
                break;
            }
        }
        for (int i = 0; i < obs_num; i++) {
            int obs_idx = cell->obs_indices[i];
            if (!ObstacleFree(obstacles[obs_idx], new_start_pt, end_pt)) {
                res_obstacles.push_back(obs_idx);

                float dist_to_end_pt = FLT_MAX;
                int obs_vertex_num = obstacles[obs_idx].vertices.size();
                for (int j = 0; j < obs_vertex_num; j++) {
                    if (SegmentIntersection(obstacles[obs_idx].vertices[j], obstacles[obs_idx].vertices[(j + 1) % obs_vertex_num],
                                            new_start_pt, end_pt)) {
                        Point2f intersetction_pt = GetSegmentsIntersectionPt(obstacles[obs_idx].vertices[j], obstacles[obs_idx].vertices[(j + 1) % obs_vertex_num],
                                                                            new_start_pt, end_pt);
                        if (cv::norm(intersetction_pt - end_pt) < dist_to_end_pt) {
                            dist_to_end_pt = cv::norm(intersetction_pt - end_pt);
                            new_start_pt = intersetction_pt;
                        }
                    }
                }
                new_start_pt += 0.5 * direction;
                start_cell_idx = LocatePtInCells(new_start_pt, cells);\
                if (res_cells.back() != start_cell_idx)
                    res_cells.push_back(start_cell_idx);
            }
        }
        
    }
    return {res_cells, res_obstacles};
}

#endif