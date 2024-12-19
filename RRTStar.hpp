#ifndef RRTSTAR_HEADER_INCLUDED
#define RRTSTAR_HEADER_INCLUDED

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
#include "obstacles.hpp"
#include "kd_tree.hpp"

using namespace cv;

class RRTStarPlanner {
public:
    Point2f start_pos_;
    Point2f target_pos_;
    std::vector<PolygonObstacle> obstacles_;
    std::vector<std::vector<int>> pv_passage_pairs_;
    std::vector<std::vector<Point2f>> pv_passage_pts_;
    std::vector<std::vector<int>> ev_passage_pairs_;
    std::vector<std::vector<Point2f>> ev_passage_pts_;
    float gamma_rrt_star_;
    float step_len_;
    float dist_to_goal_;
    int cost_function_type_;
    bool use_ev_check_;
    float passage_width_weight_;
    Mat source_img_;
    Size2f config_size_;
    PathNode* start_node_;
    PathNode* target_node_;
    kdTree kd_tree_;
    int MAX_GRAPH_SIZE = 10000;
    int GRAPH_SIZE = 0;
    int update_cost_cnt_ = 0;
    bool plan_success_ = false;

    RRTStarPlanner(): start_node_(nullptr), target_node_(nullptr) {}
    RRTStarPlanner(Point2f start, Point2f target, vector<PolygonObstacle> obs, 
                   float step_len = 20, 
                   float min_dist_to_goal = 10, 
                   Size2f config_size = Size2f(640, 480), 
                   int cost_function_type = 0,
                   float passage_width_weight = 100,
                   bool use_ev_check = true): 
        start_pos_(start), 
        target_pos_(target), 
        obstacles_(obs),
        step_len_(step_len),
        dist_to_goal_(min_dist_to_goal),
        config_size_(config_size),
        cost_function_type_(cost_function_type),
        passage_width_weight_(passage_width_weight),
        use_ev_check_(use_ev_check) {
        start_node_ = new PathNode(start);
        target_node_ = new PathNode(target);
        
        UpdateNodeCost(start_node_);
        kd_tree_.Add(start_node_);
        GRAPH_SIZE++;

        gamma_rrt_star_ = sqrt(6 * FreespaceArea(obstacles_, config_size_) / M_PI);     
        // pv: pure visibility
        auto pv_check_res = PureVisibilityPassageCheck(obstacles_);
        pv_passage_pairs_ = pv_check_res.first;
        pv_passage_pts_ = pv_check_res.second;

        // ev: extended visibility
        auto ev_check_res = ExtendedVisibilityPassageCheck(obstacles_);
        ev_passage_pairs_ = ev_check_res.first;
        ev_passage_pts_ = ev_check_res.second; 

        std::cout << "RRT* path planner instanced with cost function type: " << cost_function_type_ 
                << "\n0: Any invalid type value: Default path length cost"
                << "\n1: Ratio cost: len / passed_min_passage_width"
                << "\n2: Compound cost: len - weight * min_passed_passage_width"
                << "\n3: Compound cost: len - k_weight * k_min_passed_passage_widths"
                << "\n4: Compound cost: len - weight * passed_passage_widths"
                << "\n5: Minimum passage width cost: min_passed_passage_width"
                << "\n6: Weighted passage width sum cost: sum of weight_i * i_th_min_passed_passage_width\n\n";      
    }
    ~RRTStarPlanner();
    RRTStarPlanner(const RRTStarPlanner&);
    RRTStarPlanner& operator=(const RRTStarPlanner&);


    bool Plan(Mat source_img, float interior_delta = 0.5, bool plan_in_interior = false) {       
        srand(time(NULL));
        plan_success_ = false;
        source_img_ = source_img;
        float div_width = RAND_MAX / config_size_.width,
              div_height = RAND_MAX / config_size_.height,
              min_cost = FLT_MAX;

        Point2f rand_pos = Point2f(0, 0);
        int sample_num = 0;
        while (GRAPH_SIZE < MAX_GRAPH_SIZE) {
            // Bias to target
            sample_num++;
            if (sample_num % (MAX_GRAPH_SIZE / 20) == 0) {
                rand_pos = SafeRandTarget();                         
            } 
            else {
                rand_pos.x = rand() / div_width;
                rand_pos.y = rand() / div_height;
            }

            PathNode* nearest_node = kd_tree_.FindNearestNode(rand_pos);
            PathNode* new_node = GenerateNewNode(nearest_node, rand_pos);
            
            // cout << GRAPH_SIZE << "\n";
            if (EdgeObstacleFree(nearest_node, new_node)) {
                if (plan_in_interior && cost_function_type_ == 0)
                    if (MinDistanceToObstaclesVec(obstacles_, new_node->pos) < interior_delta) {
                        delete new_node;
                        continue;
                    }
                Rewire(nearest_node, new_node);
                kd_tree_.Add(new_node);
                /* for (auto it = new_node->sorted_passage_list.begin(); it != new_node->sorted_passage_list.end(); it++)
                    cout << *it << ", ";
                cout << "\n"; */

                if (NormSqr(new_node->pos - target_pos_) <= dist_to_goal_* dist_to_goal_) {
                    if (new_node->cost < min_cost) {
                        target_node_->parent = new_node;
                        min_cost = new_node->cost;
                    }      
                    plan_success_ = true;
                }
                GRAPH_SIZE++;
                // circle(source_img, new_node->pos, 3, Scalar(0, 255, 0), -1);
            }
            else {
                delete new_node;
            }

            /* circle(source_img, start_pos_, 10, Scalar(0, 0, 255), -1);
            circle(source_img, target_pos_, 10, Scalar(0, 0, 255), -1);
            imshow("RRT* path planning", source_img);
            waitKey(1);
            if (GRAPH_SIZE == MAX_GRAPH_SIZE)
                destroyWindow("RRT* path planning"); */
        }

        if (plan_success_ == false)
            std::cout << "MAX_GRAPH_SIZE: " << MAX_GRAPH_SIZE << " is achieved with no path found.\n";
        else
            std::cout << "Path found with cost: " << min_cost
                    << "\nTotal sample number: " << sample_num
                    << "\n";   
        return plan_success_;
    } 

    PathNode* GenerateNewNode(PathNode* nearest_node, Point2f& rand_pos) {
        float dist = cv::norm(rand_pos - nearest_node->pos);
        Point2f direction = (rand_pos - nearest_node->pos) / dist, new_pos;
        if (true)
            new_pos = nearest_node->pos + step_len_ * direction;
        else
            new_pos = rand_pos;
        new_pos.x = std::max((float)0.5, new_pos.x);
        new_pos.x = std::min(config_size_.width - (float)0.5, new_pos.x);
        new_pos.y = std::max((float)0.5, new_pos.y);
        new_pos.y = std::min(config_size_.height - (float)0.5, new_pos.y);
        PathNode* new_node = new PathNode(new_pos);
        new_node->id = GRAPH_SIZE;
        return new_node;
    }

    bool EdgeObstacleFree(PathNode* near_node, PathNode* new_node) {
        for (auto& obs : obstacles_)
            if (ObstacleFree(obs, near_node->pos, new_node->pos) == false)
                return false;
        return true;
    }

    void Rewire(PathNode* nearest_node, PathNode* new_node) {
        float gamma = gamma_rrt_star_ * sqrt(log(GRAPH_SIZE) / GRAPH_SIZE),
              radius_alg = std::min(gamma, step_len_);
        float x_min = std::max((float)0.0, new_node->pos.x - radius_alg), x_max = std::min(new_node->pos.x + radius_alg, config_size_.width),
              y_min = std::max((float)0.0, new_node->pos.y - radius_alg), y_max = std::min(new_node->pos.y + radius_alg, config_size_.height); 

        std::vector<PathNode*> near_set = kd_tree_.RanageSearch(x_min, x_max, y_min, y_max);
        int k = 0;
        for (int i = 0; i < near_set.size(); i++)
            if (EdgeObstacleFree(near_set[i], new_node) && cv::norm(near_set[i]->pos - new_node->pos) <= radius_alg) {
                near_set[k++] = near_set[i];
            }
        near_set.resize(k);

        // find minimum-cost path
        PathNode* min_cost_node = nearest_node;
        float min_cost = NewCost(nearest_node, new_node);
        for (auto near_node : near_set) {
            float cur_cost = NewCost(near_node, new_node);       
            if (cur_cost < min_cost - 1e-2) {
                min_cost_node = near_node;
                min_cost = cur_cost;
            }            
        }
        UpdateSubtree(min_cost_node, new_node); 
        // line(source_img, min_cost_node->pos, new_node->pos, Scalar(0, 0, 200), 1.5);

        for (auto near_node : near_set) {
            if (near_node == min_cost_node)
                continue;

            float new_near_node_cost = NewCost(new_node, near_node);
            if (new_near_node_cost < near_node->cost 
                || (new_near_node_cost < near_node->cost + 1e-2
                   && new_node->len + cv::norm(near_node->pos - new_node->pos) < near_node->len)
                ) {
                UpdateSubtree(new_node, near_node);
            }
        }
    }

    float NewCost(PathNode* near_node, PathNode* new_node) {
        std::list<float> new_psg_list = near_node->sorted_passage_list;
        float new_len = near_node->len + cv::norm(new_node->pos - near_node->pos),
              passed_psg_width = -1.0, 
              new_min_psg_width = near_node->min_passage_width,
              res = 0;
        if (cost_function_type_ == 0)
            return new_len;

        if (use_ev_check_ == true) 
            passed_psg_width = GetMinPassageWidthPassed(ev_passage_pts_, near_node->pos, new_node->pos);
        else 
            passed_psg_width = GetMinPassageWidthPassed(pv_passage_pts_, near_node->pos, new_node->pos);

        if (passed_psg_width > 0) {
            new_min_psg_width = std::min(new_min_psg_width, passed_psg_width);
            InsertIntoSortedList(new_psg_list, passed_psg_width);
        }
        
        if (cost_function_type_ == 1) {
            res = new_len / new_min_psg_width;
        } 
        else if (cost_function_type_ == 2) {
            // res = new_len - passage_width_weight_ * new_min_psg_width;
            res = -new_min_psg_width;
        }
        return res;      
    }    

    void UpdateNodeCost(PathNode* node) {
        if (cost_function_type_ == 0) {
            node->cost = node->len;
        }            
        else if (cost_function_type_ == 1) {
            node->cost = node->len / node->min_passage_width;
        } 
        else if (cost_function_type_ == 2) {
            // node->cost = node->len - passage_width_weight_ * node->min_passage_width;
            node->cost = -node->min_passage_width;
        }   
    }

    void UpdateSubtree(PathNode* new_parent, PathNode* child) {
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
        if (use_ev_check_ == true) 
            passed_psg_width = GetMinPassageWidthPassed(ev_passage_pts_, new_parent->pos, child->pos);
        else 
            passed_psg_width = GetMinPassageWidthPassed(pv_passage_pts_, new_parent->pos, child->pos);

        if (passed_psg_width > 0) {  
            InsertIntoSortedList(child->sorted_passage_list, passed_psg_width); 
            child->cur_passage_width = passed_psg_width;
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
                    InsertIntoSortedList(cur_child->sorted_passage_list, cur_child->cur_passage_width);
                }
                UpdateNodeCost(cur_child);
                node_level.push(cur_child);
            }
        }
    }

    /// Instead of directly using rand_pos = target_pos_, add a small random shift to avoid resulting in
    /// samples in the same position and potentially any further problematic circle connection in the graph.
    Point2f SafeRandTarget() {
        float safe_dist_x = min(target_pos_.x, config_size_.width - target_pos_.x),
              safe_dist_y = min(target_pos_.y, config_size_.height - target_pos_.y),
              safe_dist = min(safe_dist_x, safe_dist_y) * 0.95;
        float r = sqrt(rand() / (float)RAND_MAX) * safe_dist;
        Point2f direction;
        direction.x = rand() / (float)RAND_MAX; direction.y = rand() / (float)RAND_MAX;
        direction = direction - Point2f(0.5, 0.5);
        direction = direction / cv::norm(direction);
        return target_pos_ + direction * r;
    }

    std::vector<PathNode*> GetPath() {
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

    std::vector<Point2f> GetPathInPts() {
        std::vector<PathNode*> node_path = GetPath();
        std::vector<Point2f> res(node_path.size());

        for (int i = 0; i < node_path.size(); i++)
            res[i] = node_path[i]->pos;
        return res;
    }
};

RRTStarPlanner::~RRTStarPlanner() {
    // delete start_node_;
    // delete target_node_;
}

RRTStarPlanner::RRTStarPlanner(const RRTStarPlanner& planner) {
    start_pos_ = planner.start_pos_;
    target_pos_ = planner.target_pos_;
    if (!start_node_)
        start_node_ = new PathNode(start_pos_);
    else {
        kd_tree_.deleteTree(start_node_);
        start_node_ = new PathNode(start_pos_);
        kd_tree_.kd_tree_root_ = start_node_;
    }
    if (!target_node_)
        target_node_ = new PathNode(target_pos_);
    else {
        delete target_node_;
        target_node_ = new PathNode(target_pos_);
    }
    obstacles_ = planner.obstacles_;
    step_len_ = planner.step_len_;
    dist_to_goal_ = planner.dist_to_goal_;
    config_size_ = planner.config_size_;
    MAX_GRAPH_SIZE = planner.MAX_GRAPH_SIZE;
    GRAPH_SIZE = planner.GRAPH_SIZE;
    plan_success_ = planner.plan_success_;        
}

RRTStarPlanner& RRTStarPlanner::operator=(const RRTStarPlanner& rhs) {
    start_pos_ = rhs.start_pos_;
    target_pos_ = rhs.target_pos_;
    if (!start_node_)
        start_node_ = new PathNode(start_pos_);
    else {
        kd_tree_.deleteTree(start_node_);
        start_node_ = new PathNode(start_pos_);
        kd_tree_.kd_tree_root_ = start_node_;
    }
    if (!target_node_)
        target_node_ = new PathNode(target_pos_);
    else {
        delete target_node_;
        target_node_ = new PathNode(target_pos_);
    }
    
    obstacles_ = rhs.obstacles_;
    step_len_ = rhs.step_len_;
    dist_to_goal_ = rhs.dist_to_goal_;
    config_size_ = rhs.config_size_;    
    MAX_GRAPH_SIZE = rhs.MAX_GRAPH_SIZE;
    GRAPH_SIZE = rhs.GRAPH_SIZE;
    plan_success_ = rhs.plan_success_;
    return *this;
}
#endif