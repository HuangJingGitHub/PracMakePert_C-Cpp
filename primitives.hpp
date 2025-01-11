#ifndef PRIMITIVES_INCLUDED
#define PRIMITIVES_INCLUDED

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

bool EdgeObstacleFree(PathNode* near_node, PathNode* new_node, const vector<PolygonObstacle>& obstacles) {
    for (auto& obs : obstacles)
        if (ObstacleFree(obs, near_node->pos, new_node->pos) == false)
            return false;
    return true;
}

float NewCost(PathNode* near_node, PathNode* new_node,                       
            vector<PolygonCell>& cells_info, 
            unordered_map<string, vector<int>> psg_cell_idx_map,
            const int cost_function_type = 0) {
    std::list<float> new_psg_list = near_node->sorted_passage_list;
    float new_len = near_node->len + cv::norm(new_node->pos - near_node->pos), 
            new_min_psg_width = near_node->min_passage_width,
            res = 0;
    vector<float> passed_width_vec;
    if (cost_function_type == 0)
        return new_len;

    // passed_width_vec = GetPassageWidthsPassed(ev_passage_pts_, near_node->pos, new_node->pos);
    passed_width_vec = GetPassageWidthsPassedDG(near_node, new_node, cells_info, psg_cell_idx_map, false);
    if (passed_width_vec.size() > 0) {
        for (float& psg_width : passed_width_vec)
            new_min_psg_width = std::min(new_min_psg_width, psg_width);
        InsertIntoSortedList(new_psg_list, passed_width_vec);                
    }
    
    if (cost_function_type == 1) {
        res = new_len / new_min_psg_width;
    } 
    else if (cost_function_type == 2) {
        res = -new_min_psg_width;
    }
    else if (cost_function_type == 3) {
        vector<float> base{1e4, 1e2, 1};
        auto it = new_psg_list.begin();
        if (new_psg_list.size() < 3) {
            int i = 0;
            while (i < new_psg_list.size()) {
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

void UpdateNodeCost(PathNode* node, const int cost_function_type = 0) {
    if (cost_function_type == 0) {
        node->cost = node->len;
    }            
    else if (cost_function_type == 1) {
        node->cost = node->len / node->min_passage_width;
    } 
    else if (cost_function_type == 2) {
        node->cost = -node->min_passage_width;
    }   
    else if (cost_function_type == 3) {
        node->cost = 0;
        vector<float> base{1e4, 1e2, 1};
        auto it = node->sorted_passage_list.begin();
        if (node->sorted_passage_list.size() < 3) {
            int i = 0;
            while (i < node->sorted_passage_list.size()) {
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

void UpdateSubtree(PathNode* new_parent, PathNode* child,
                    vector<PolygonCell>& cells_info, 
                    unordered_map<string, vector<int>> psg_cell_idx_map) {
    PathNode* old_parent = child->parent;
    if (old_parent != nullptr) 
        old_parent->children.remove(child);
    child->parent = new_parent;
    new_parent->children.push_back(child);

    float old_len = child->len, len_change = 0;
    child->len = new_parent->len + cv::norm(new_parent->pos - child->pos);
    len_change = child->len - old_len;

    child->min_passage_width = new_parent->min_passage_width;
    child->cur_passage_widths.clear();
    child->sorted_passage_list = new_parent->sorted_passage_list;

    vector<float> passed_width_vec;
    // passed_width_vec = GetPassageWidthsPassed(ev_passage_pts_, new_parent->pos, child->pos);
    passed_width_vec = GetPassageWidthsPassedDG(new_parent, child, cells_info, psg_cell_idx_map, true);
    if (passed_width_vec.size() > 0) {
        child->cur_passage_widths = passed_width_vec;
        InsertIntoSortedList(child->sorted_passage_list, passed_width_vec); 
        for (float& psg_width : passed_width_vec)
            child->min_passage_width = std::min(child->min_passage_width, psg_width);
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
            if (cur_child->cur_passage_widths.size() > 0) {
                for (float& psg_width : cur_child->cur_passage_widths)
                    cur_child->min_passage_width = std::min(cur_node->min_passage_width, psg_width);
                InsertIntoSortedList(cur_child->sorted_passage_list, cur_child->cur_passage_widths);
            }
            UpdateNodeCost(cur_child);
            node_level.push(cur_child);
        }
    }
}

vector<PathNode*> GetPath(const bool plan_success, PathNode* target_node) {
    vector<PathNode*> res;
    if (!plan_success) {
        std::cout << "No valid path is available.\n";
        return res;
    }
    PathNode* reverse_node = target_node;
    while (reverse_node) {
        res.push_back(reverse_node);
        reverse_node = reverse_node->parent;
    }
    reverse(res.begin(), res.end());
    return res;   
}  

vector<Point2f> GetPathInPts(const bool plan_success, PathNode* target_node) {
    vector<PathNode*> node_path = GetPath(plan_success, target_node);
    vector<Point2f> res(node_path.size());

    for (int i = 0; i < node_path.size(); i++)
        res[i] = node_path[i]->pos;
    return res;
}
#endif