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
    int MAX_GRAPH_SIZE = 100;
    std::vector<PathNode*> node_vec_ = vector<PathNode*>(MAX_GRAPH_SIZE);
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
        node_vec_[GRAPH_SIZE++] = start_node_;

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
    
    void ConstructRoadmap(Mat source_img);
    void QueryPath(Mat source_img);
    void UpdateNodeCost(PathNode* node);
    float NewCost(PathNode* near_node, PathNode* new_node);
    void UpdateSubtree(PathNode* new_parent, PathNode* child);
    bool EdgeObstacleFree(PathNode* near_node, PathNode* new_node);
    vector<PathNode*> GetPath();
};

void PRMStarPlanner::ConstructRoadmap(Mat source_img) {
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
        new_node->cost = 1e8;
        new_node->len = 1e8;
        new_node->id = GRAPH_SIZE;
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
        vector<PathNode*> near_set = kd_tree_.RanageSearch(x_min, x_max, y_min, y_max);
        for (auto near_node : near_set) {
            if (EdgeObstacleFree(node, near_node) == false 
                || cv::norm(node->pos - near_node->pos) > radius_alg
                || near_node == node)
                continue;
            node->adjacency_list.push_back(near_node);
            // near_node->adjacency_list.push_back(node);
            // line(source_img, node->pos, near_node->pos, Scalar(0, 0, 255), 1);
            // imshow("PRM* for PTOPP samples", source_img);
            // waitKey(1);                
        }                             
    }
}

void PRMStarPlanner::QueryPath(Mat source_img) {
    if (GRAPH_SIZE < MAX_GRAPH_SIZE)
        ConstructRoadmap(source_img);
    
    vector<bool> visited(node_vec_.size(), false);
    UpdateNodeCost(start_node_);
    float min_cost = FLT_MAX;
    int min_cost_idx = 0;
    PathNode* target_graph_node = kd_tree_.FindNearestNode(target_node_);
    target_node_->parent = target_graph_node;
    priority_queue<PathNode*, vector<PathNode*>, PathNodeComparator> min_cost_heap;
    min_cost_heap.push(start_node_);

    for (int i = 0; i < node_vec_.size(); i++) {
        min_cost = FLT_MAX;
        for (int j = 0; j < node_vec_.size(); j++) {
            if (visited[j] == false && (node_vec_[j]->cost < min_cost
                || (node_vec_[j]->cost < min_cost + 1e-2 && node_vec_[j]->len < node_vec_[min_cost_idx]->len))) {
                min_cost = node_vec_[j]->cost;
                min_cost_idx = j;
            }
        }  
        PathNode* node = node_vec_[min_cost_idx];
        if (node == target_graph_node)
            break;
        while (min_cost_heap.empty() == false && visited[min_cost_heap.top()->id] == true)
            min_cost_heap.pop();
        PathNode* top_node = min_cost_heap.top();
        min_cost_heap.pop();
        visited[node->id] = true;
        cout << node->id << "-" << top_node->id << "\n";
        for (auto adj_node : node->adjacency_list) {
            if (visited[adj_node->id] == true)
                continue;

            float temp_cost = NewCost(node, adj_node);
            if (temp_cost < adj_node->cost 
                || (temp_cost < adj_node->cost + 1e-2 
                    && node->len + cv::norm(adj_node->pos - node->pos) < adj_node->len)) {
                UpdateSubtree(node, adj_node);
                min_cost_heap.push(adj_node);
            }
        }
    }
    plan_success_ = true;
}

void PRMStarPlanner::UpdateNodeCost(PathNode* node) {
    if (cost_function_type_ == 0) {
        node->cost = node->len;
    }            
    else if (cost_function_type_ == 1) {
        node->cost = node->len / node->min_passage_width;
    } 
    else if (cost_function_type_ == 2) {
        node->cost = -node->min_passage_width;
    }   
    else if (cost_function_type_ == 3) {
        vector<float> base{1e4, 1e2, 1};
        int list_size = node->sorted_passage_list.size();
        auto it = node->sorted_passage_list.begin();
        if (list_size < 3) {
            int i = 0;
            while (i < list_size) {
                node->cost -= base[i] * (*it);
                i++; it++;
            }
            while (i < 3) {
                node->cost -= base[i] * 1000;
                i++;
            }
        }
        else {
            for (int i = 0; i < 3; i++, it++)
                node->cost -= base[i] * (*it);
        }         
    }
}

float PRMStarPlanner::NewCost(PathNode* near_node, PathNode* new_node) {
    std::list<float> new_psg_list = near_node->sorted_passage_list;
    float new_len = near_node->len + cv::norm(new_node->pos - near_node->pos),
          passed_psg_width = -1.0, 
          new_min_psg_width = near_node->min_passage_width,
          res = 0;
    vector<float> passed_width_vec;
    if (cost_function_type_ == 0)
        return new_len;

    // passed_width_vec = GetPassageWidthsPassed(ev_passage_pts_, near_node->pos, new_node->pos);
    passed_width_vec = GetPassageWidthsPassedDG(near_node, new_node, cells_info_, psg_cell_idx_map_, false);
    if (passed_width_vec.size() > 0) {
        passed_psg_width = passed_width_vec[0];
        for (int i = 1 ; i < passed_width_vec.size(); i++)
            passed_psg_width = min(passed_psg_width, passed_width_vec[i]);
    }
    if (passed_psg_width > 0) {
        new_min_psg_width = std::min(new_min_psg_width, passed_psg_width);
        InsertIntoSortedList(new_psg_list, passed_width_vec);
    }
    
    if (cost_function_type_ == 1) {
        res = new_len / new_min_psg_width;
    } 
    else if (cost_function_type_ == 2) {
        res = -new_min_psg_width;
    }
    else if (cost_function_type_ == 3) {
        vector<float> base{1e4, 1e2, 1};
        int list_size = new_psg_list.size();
        auto it = new_psg_list.begin();
        if (list_size < 3) {
            int i = 0;
            while (i < list_size) {
                res -= base[i] * (*it);
                i++; it++;
            }
            while (i < 3) {
                res -= base[i] * 1000;
                i++;
            }
        }
        else {
            for (int i = 0; i < 3; i++, it++) 
                res -= base[i] * (*it);
        } 
    }
    return res;      
} 

void PRMStarPlanner::UpdateSubtree(PathNode* new_parent, PathNode* child) {
    PathNode* old_parent = child->parent;
    if (old_parent != nullptr) 
        old_parent->children.remove(child);
    child->parent = new_parent;
    new_parent->children.push_back(child);

    float old_len = child->len, len_change = 0;
    child->len = new_parent->len + cv::norm(new_parent->pos - child->pos);
    len_change = child->len - old_len;

    child->min_passage_width = new_parent->min_passage_width;
    child->sorted_passage_list = new_parent->sorted_passage_list;

    float passed_psg_width = -1.0;
    vector<float> passed_width_vec;
    // passed_width_vec = GetPassageWidthsPassed(ev_passage_pts_, new_parent->pos, child->pos);
    passed_width_vec = GetPassageWidthsPassedDG(new_parent, child, cells_info_, psg_cell_idx_map_, true);
    if (passed_width_vec.size() > 0) {
        passed_psg_width = passed_width_vec[0];
        for (int i = 1; i < passed_width_vec.size(); i++)
            passed_psg_width = min(passed_psg_width, passed_width_vec[i]);
    }

    if (passed_psg_width > 0) {  
        InsertIntoSortedList(child->sorted_passage_list, passed_width_vec); 
        child->cur_passage_width = passed_psg_width;
        child->cur_passage_widths = passed_width_vec;
        child->min_passage_width = std::min(child->min_passage_width, passed_psg_width);          
    }
    UpdateNodeCost(child); 

    std::queue<PathNode*> node_level;
    node_level.push(child);
    int loop_num = 0;
    while (node_level.empty() == false) {
        PathNode* cur_node = node_level.front();
        node_level.pop();

        loop_num++;
        if (loop_num > 1 && cur_node->id == child->id) {
            throw std::invalid_argument("Circle detected in the tree");
        }
        
        for (auto& cur_child : cur_node->children) {
            cur_child->len += len_change;
            cur_child->min_passage_width = cur_node->min_passage_width;
            cur_child->sorted_passage_list = cur_node->sorted_passage_list;
            if (cur_child->cur_passage_width > 0) {
                cur_child->min_passage_width = std::min(cur_node->min_passage_width, cur_child->cur_passage_width);
                InsertIntoSortedList(cur_child->sorted_passage_list, cur_child->cur_passage_widths);
            }
            UpdateNodeCost(cur_child);
            node_level.push(cur_child);
        }
    }
}

bool PRMStarPlanner::EdgeObstacleFree(PathNode* near_node, PathNode* new_node) {
    for (auto& obs : obstacles_)
        if (ObstacleFree(obs, near_node->pos, new_node->pos) == false)
            return false;
    return true;
}

vector<PathNode*> PRMStarPlanner::GetPath() {
    std::vector<PathNode*> res;
    if (!plan_success_) {
        std::cout << "No valid path is available.\n";
        return res;
    }
    PathNode* reverse_node = target_node_;
    while (reverse_node) {
        res.push_back(reverse_node);
        reverse_node = reverse_node->parent;
    }
    reverse(res.begin(), res.end());
    return res;   
} 

PRMStarPlanner::~PRMStarPlanner() {
    if (start_node_) {
        kd_tree_.deleteTree(start_node_);
    }
    if (target_node_)
        delete target_node_;
}

PRMStarPlanner::PRMStarPlanner(const PRMStarPlanner& planner) {
      
}

PRMStarPlanner& PRMStarPlanner::operator=(const PRMStarPlanner& rhs) {
    return *this;
}
#endif