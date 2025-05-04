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
    vector<vector<float>> intersect_psg_heights;
    bool is_an_obstacle = false;
    PolygonCell3d() {}
    PolygonCell3d(vector<int> cell_obs_indices, vector<Point2f> cell_vertices): obs_indices(cell_obs_indices), vertices(cell_vertices) {}
    PolygonCell3d(PolygonObstacle3d obs, int obs_idx): obs_indices({obs_idx}), vertices(obs.vertices), is_an_obstacle(true) {}
};

int LocatePtInGivenCells(const Point2f pt, const vector<PolygonCell3d>& cell_info, const vector<int>& given_indices) {
    for (int i : given_indices) {
        if (InsidePolygon(pt, cell_info[i].vertices))
            return i;
    }
    return -1;
}

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

vector<vector<int>> GetPassedCellsAndObstacles(const Point2f& start_pt, const Point2f& end_pt,
                                                const vector<PolygonCell>& cells, const vector<PolygonObstacle>& obstacles,
                                                Mat& back_img, int start_obs_idx = 0, int end_obs_idx = 0) {
    vector<int> res_cells, res_obstacles;
    unordered_map<string, vector<int>> psg_cell_idx_map = GetPassageCellMap(cells);

    Point2f direction = (end_pt - start_pt) / cv::norm(end_pt - start_pt),
            new_start_pt = start_pt + 0.1 * direction,
            new_end_pt = end_pt - 0.1 * direction;
    int start_cell_idx = LocatePtInCells(new_start_pt, cells),
        end_cell_idx = LocatePtInCells(new_end_pt, cells);
    res_cells.push_back(start_cell_idx);
    set<string> passed_psg_pairs;

    while (start_cell_idx != end_cell_idx) {
        circle(back_img, start_pt, 10, Scalar(0, 0, 0));
        circle(back_img, end_pt, 10, Scalar(0, 0, 0));
        imshow("Obstacles on Base Ground", back_img);

        cout << "\nstart and end cell indices: " << start_cell_idx << "-" << end_cell_idx << "\n";
        cout << start_pt << "-" << end_pt << "\n";
        cout << start_obs_idx << "-" << end_obs_idx << "\n";
        if (start_cell_idx == -1 || end_cell_idx == -1)
            waitKey(0);
        else 
            waitKey(50); 

        const PolygonCell* cell = &cells[start_cell_idx];
        int obs_num = cell->obs_indices.size();
        for (int i = 0; i < obs_num; i++) {
            Point2f vertex_1 = cell->vertices[2 * i],
                    vertex_2 = cell->vertices[2 * i + 1];
            if (SegmentIntersection(vertex_1, vertex_2, new_start_pt, new_end_pt)) {
                string key_str = PassagePairToKeyString(cell->obs_indices[i], cell->obs_indices[(i + 1) % obs_num]);
                if (passed_psg_pairs.count(key_str) > 0) {
                    continue;
                }

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
            if (!ObstacleFree(obstacles[obs_idx], new_start_pt, new_end_pt)) {
                res_obstacles.push_back(obs_idx);

                float dist_to_end_pt = FLT_MAX;
                Point2f nearest_intersetction_pt;
                int obs_vertex_num = obstacles[obs_idx].vertices.size();
                for (int j = 0; j < obs_vertex_num; j++) {
                    if (SegmentIntersection(obstacles[obs_idx].vertices[j], obstacles[obs_idx].vertices[(j + 1) % obs_vertex_num],
                                            new_start_pt, new_end_pt)) {
                        Point2f intersetction_pt = GetSegmentsIntersectionPt(obstacles[obs_idx].vertices[j], 
                                                                            obstacles[obs_idx].vertices[(j + 1) % obs_vertex_num],
                                                                            new_start_pt, new_end_pt);
                        if (cv::norm(intersetction_pt - end_pt) < dist_to_end_pt) {
                            dist_to_end_pt = cv::norm(intersetction_pt - end_pt);
                            nearest_intersetction_pt = intersetction_pt;
                        }
                    }
                }
                // new_start_pt += 0.1 * direction;
                int current_cell_idx = LocatePtInCells(nearest_intersetction_pt + 0.1 * direction, cells);

                // current_cell_idx is not necessiarly a new cell not passed. 
                // If it is passed and assigned to start_cell_idx, infinite loop will happen.
                bool is_new_cell = true;
                for (int passed_cell_idx : res_cells) {
                    if (passed_cell_idx == current_cell_idx) {
                        is_new_cell = false;
                        break;
                    }
                }
                if (is_new_cell) {
                    res_cells.push_back(current_cell_idx);
                    start_cell_idx = current_cell_idx;
                }
            }
        }
        
    }
    return {res_cells, res_obstacles};
}

vector<PolygonCell3d> GetCompoundGabrielCells3d(const vector<PolygonObstacle3d>& obstacles, const Passages3d& passages_3d, Mat& back_img) {
    // Passages3d passages_3d = PassageCheckDelaunayGraphWithWalls3d(obstacles);

    vector<PolygonObstacle> obstacles_2d = ConvertTo2dObstacles(obstacles);
    Passages passages_2d = PassageCheckDelaunayGraphWithWalls(obstacles_2d);

    vector<vector<int>> cell_faces = ReportGabrielCells(obstacles_2d, passages_2d.pairs, true);
    vector<PolygonCell> cells_2d = GetGabrielCellsInfo(cell_faces, passages_2d);
    
    vector<PolygonCell3d> res(cells_2d.size() + obstacles.size());
    for (int i = 0; i < cells_2d.size(); i++)
        res[i] = PolygonCell3d(cells_2d[i].obs_indices, cells_2d[i].vertices);
    // Walls are added, but will never be used as position holders.    
    for (int i = 0; i < obstacles.size(); i++)
        res[cells_2d.size() + i] = PolygonCell3d(obstacles[i], i);
    for (int i = 0; i < passages_3d.pairs.size(); i++) {
        if (passages_3d.heights[i][0] < 1e-2)
            continue;

        vector<vector<int>> passed_cell_obs = GetPassedCellsAndObstacles(passages_3d.pts[i][0], passages_3d.pts[i][1], 
                                                                        cells_2d, obstacles_2d, back_img, 
                                                                        passages_3d.pairs[i][0], passages_3d.pairs[i][1]);
        for (int cell_id : passed_cell_obs[0]) {
            res[cell_id].intersect_psg_pairs.push_back(passages_3d.pairs[i]);
            res[cell_id].intersect_psg_pts.push_back(passages_3d.pts[i]);
            res[cell_id].intersect_psg_heights.push_back(passages_3d.heights[i]);
        }
        for (int obs_id : passed_cell_obs[1]) {
            res[cells_2d.size() + obs_id].intersect_psg_pairs.push_back(passages_3d.pairs[i]);
            res[cells_2d.size() + obs_id].intersect_psg_pts.push_back(passages_3d.pts[i]); 
            res[cells_2d.size() + obs_id].intersect_psg_heights.push_back(passages_3d.heights[i]);
        }
    }
    return res;
}

vector<float> GetPassedPassageWidthsDG3d(PathNode3d* start_node, PathNode3d* end_node,
                                       const vector<PolygonCell3d>& cells, 
                                       Passages3d passages_3d,
                                       bool update_end_node_cell_idx = false) {
    vector<float> res;
    set<string> passed_psg_pairs;
    int start_cell_idx = start_node->cell_id;
    Point3f start_pt = start_node->pos, end_pt = end_node->pos;
    Point2f start_pt_2d(start_pt.x, start_pt.y), end_pt_2d(end_pt.x, end_pt.y),
            direction = (end_pt_2d - start_pt_2d) / cv::norm(end_pt_2d - start_pt_2d);
    unordered_map<string, vector<int>> psg_cell_idx_map;
    unordered_map<int, vector<int>> obs_cell_idx_map;
    unordered_map<string, vector<float>> psg_heights_map;

    for (int i = 0; i < cells.size(); i++) {
        if (cells[i].is_an_obstacle)
            continue;

        int vertex_num = cells[i].obs_indices.size();
        for (int j = 0; j < vertex_num; j++) {
            string key_str = PassagePairToKeyString(cells[i].obs_indices[j], cells[i].obs_indices[(j + 1) % vertex_num]);
            psg_cell_idx_map[key_str].push_back(i);
            obs_cell_idx_map[j].push_back(i);
        }
    }
    for (int i = 0; i < passages_3d.pairs.size(); i++) {
        string key_string = PassagePairToKeyString(passages_3d.pairs[i][0], passages_3d.pairs[i][1]);
        psg_heights_map[key_string] = passages_3d.heights[i];
    }

    bool end_pt_located = false;
    while (!end_pt_located) {
        end_pt_located = true;
        const PolygonCell3d* cell = &cells[start_cell_idx];
        if (!cell->is_an_obstacle) {
            int side_num = cell->obs_indices.size();
            for (int i = 0; i < side_num; i++) {
                Point2f vertex_1 = cell->vertices[2 * i],
                        vertex_2 = cell->vertices[2 * i + 1];
                string key_str = PassagePairToKeyString(cell->obs_indices[i], cell->obs_indices[(i + 1) % side_num]);
                if (SegmentIntersection(vertex_1, vertex_2, start_pt_2d, end_pt_2d)) {
                    if (passed_psg_pairs.count(key_str) > 0)
                        continue;
                    
                    if (PlaneIntersection(vertex_1, vertex_2, psg_heights_map[key_str][0], psg_heights_map[key_str][1], start_pt, end_pt))
                        res.push_back(cv::norm(vertex_1 - vertex_2));

                    passed_psg_pairs.insert(key_str);
                    for (int cell_idx : psg_cell_idx_map[key_str]) {
                        if (cell_idx != start_cell_idx) {
                            start_cell_idx = cell_idx;
                            end_pt_located = false;
                            break;
                        }
                    }
                    break;
                }
            }

            for (int i = 0; i < cell->intersect_psg_pairs.size(); i++) {
                if (PlaneIntersection(cell->intersect_psg_pts[i][0], cell->intersect_psg_pts[i][1], 
                                      cell->intersect_psg_heights[i][0], cell->intersect_psg_heights[i][1],
                                      start_pt, end_pt)) 
                    res.push_back(cv::norm(cell->intersect_psg_pts[i][0] - cell->intersect_psg_pts[i][1]));
            }
        }
        else {
            int obs_idx = cell->obs_indices[0];
            for (int i = 0; i < cell->intersect_psg_pairs.size(); i++) {
                if (PlaneIntersection(cell->intersect_psg_pts[i][0], cell->intersect_psg_pts[i][1],
                                      cell->intersect_psg_heights[i][0], cell->intersect_psg_heights[i][1],
                                      start_pt, end_pt))
                    res.push_back(cv::norm(cell->intersect_psg_pts[i][0] - cell->intersect_psg_pts[i][1]));
            }
            int vertex_num = cell->vertices.size();
            for (int i = 0; i < vertex_num; i++) {
                if (SegmentIntersection(cell->vertices[i], cell->vertices[(i + 1) % vertex_num], start_pt_2d, end_pt_2d)) {
                    Point2f intersection_pt = GetSegmentsIntersectionPt(cell->vertices[i], cell->vertices[(i + 1) % vertex_num],
                                                                        start_pt_2d, end_pt_2d);
                    intersection_pt += 0.1 * direction;
                    int next_cell_index = LocatePtInGivenCells(intersection_pt, cells, obs_cell_idx_map[obs_idx]);
                }
            }
        }
    }
    if (update_end_node_cell_idx)
        end_node->cell_id = start_cell_idx;
    return res;
}

#endif