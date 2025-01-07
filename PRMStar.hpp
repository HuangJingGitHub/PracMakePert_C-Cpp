#ifndef PRMSTAR_HEADER_INCLUDED
#define PRMSTAR_HEADER_INCLUDED

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "decomposition.hpp"

using namespace cv;

class PRMStarPlanner {
public:
    Point2f start_pos_;
    Point2f target_pos_;
    std::vector<PolygonObstacle> obstacles_;
    std::vector<std::vector<int>> ev_passage_pairs_;
    std::vector<std::vector<Point2f>> ev_passage_pts_;
    std::vector<std::vector<int>> cells_;
    std::vector<PolygonCell> cells_info_;
    std::unordered_map<std::string, std::vector<int>> psg_cell_idx_map_;
    float gamma_prm_star_;
    int cost_function_type_;
    bool use_ev_check_;
    float passage_width_weight_;
    Mat source_img_;
    Size2f config_size_;
    PathNode* start_node_;
    PathNode* target_node_;
    kdTree kd_tree_;
    int MAX_GRAPH_SIZE = 5000;
    vector<PathNode*> node_vec_ = vector<PathNode*>(MAX_GRAPH_SIZE);
    int GRAPH_SIZE = 0;
    bool plan_success_ = false;

    PRMStarPlanner(): start_node_(nullptr), target_node_(nullptr) {}
    PRMStarPlanner(Point2f start, Point2f target, vector<PolygonObstacle> obs,
                   Size2f config_size = Size2f(640, 480), 
                   int cost_function_type = 0,
                   float passage_width_weight = 100,
                   bool use_ev_check = true): 
        start_pos_(start), 
        target_pos_(target), 
        obstacles_(obs),
        config_size_(config_size),
        cost_function_type_(cost_function_type),
        passage_width_weight_(passage_width_weight),
        use_ev_check_(use_ev_check) {
        start_node_ = new PathNode(start);
        target_node_ = new PathNode(target);
        
        // kd_tree_.Add(start_node_);
        node_vec_[GRAPH_SIZE] = start_node_;
        GRAPH_SIZE++;

        gamma_prm_star_ = sqrt(6 * FreespaceArea(obstacles_, config_size_) / M_PI);

        // Extended visibility as Gabriel condition in Delaunay graph
        auto DG_check_res = PassageCheckDelaunayGraphWithWalls(obstacles_);
        ev_passage_pairs_ = DG_check_res.first;
        ev_passage_pts_ = DG_check_res.second; 

        cells_ = ReportGabrielCells(obstacles_, ev_passage_pairs_);
        cells_info_ = GetGabrielCellsInfo(cells_, DG_check_res);
        psg_cell_idx_map_ = GetPassageCellMap(cells_info_);
        start_node_->cell_id = LocatePtInCells(start_node_->pos, cells_info_);

        std::cout << "PRM* path planner instanced with cost function type: " << cost_function_type_ 
                << "\n0: Any invalid type value: Default path length cost"
                << "\n1: Ratio cost: len / passed_min_passage_width"
                << "\n2: Compound cost: len - weight * min_passed_passage_width"
                << "\n3: Compound cost: len - k_weight * k_min_passed_passage_widths"
                << "\n4: Compound cost: len - weight * passed_passage_widths"
                << "\n5: Minimum passage width cost: min_passed_passage_width"
                << "\n6: Weighted passage width sum cost: sum of weight_i * i_th_min_passed_passage_width\n\n";      
    }
    ~PRMStarPlanner();
    PRMStarPlanner(const PRMStarPlanner&);
    PRMStarPlanner& operator=(const PRMStarPlanner&);

    void ConstructRoadmap(Mat source_img) {
        srand(time(NULL));
        float div_width = RAND_MAX / config_size_.width,
              div_height = RAND_MAX / config_size_.height;
        Point2f rand_pos(0, 0);
        while (GRAPH_SIZE < MAX_GRAPH_SIZE) {
            rand_pos.x = rand() / div_width;
            rand_pos.y = rand() / div_height;
            if (PointFreeofObstacles(rand_pos, obstacles_) == false)
                continue;
            
            PathNode* new_node = new PathNode(rand_pos);
            new_node->len = 1e6;
            node_vec_[GRAPH_SIZE++] = new_node;
            circle(source_img, rand_pos, 3, Scalar(0, 255, 0), -1);
        }

        for (auto node : node_vec_)
            kd_tree_.Add(node);
        float radius_alg = gamma_prm_star_ * sqrt(log(MAX_GRAPH_SIZE) / MAX_GRAPH_SIZE);
        for (auto node : node_vec_) {
            float x_min = std::max((float)0.0, node->pos.x - radius_alg), 
                  x_max = std::min(node->pos.x + radius_alg, config_size_.width),
                  y_min = std::max((float)0.0, node->pos.y - radius_alg), 
                  y_max = std::min(node->pos.y + radius_alg, config_size_.height);  
            std::vector<PathNode*> near_set = kd_tree_.RanageSearch(x_min, x_max, y_min, y_max);
            for (auto near_node : near_set) {
                if (EdgeObstacleFree(node, near_node) == false 
                    || cv::norm(node->pos - near_node->pos) > radius_alg
                    || near_node == node)
                    continue;
                node->adjacency_list.push_back(near_node);
                // near_node->adjacency_list.push_back(node);
                line(source_img, node->pos, near_node->pos, Scalar(0, 0, 255), 1);
                imshow("PRM* for PTOPP samples", source_img);
                waitKey(1);                
            }                             
        }
    }

    bool EdgeObstacleFree(PathNode* near_node, PathNode* new_node) {
    for (auto& obs : obstacles_)
        if (ObstacleFree(obs, near_node->pos, new_node->pos) == false)
            return false;
    return true;
    }
 
};

PRMStarPlanner::~PRMStarPlanner() {
    // delete start_node_;
    // delete target_node_;
}

PRMStarPlanner::PRMStarPlanner(const PRMStarPlanner& planner) {
      
}

PRMStarPlanner& PRMStarPlanner::operator=(const PRMStarPlanner& rhs) {
    return *this;
}
#endif