#ifndef RRTSTAR_HEADER_INCLUDED
#define RRTSTAR_HEADER_INCLUDED

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "obstacles.h"

using namespace cv;
using namespace std;


struct RRTStarNode {
    Point2f pos;
    float cost;
    float min_passage_width = 500;
    RRTStarNode* parent;
    vector<RRTStarNode*> adjacency_list;
    RRTStarNode(): pos(Point2f(0, 0)), cost(0), parent(nullptr) {}
    RRTStarNode(Point2f initPos): pos(initPos), cost(0), parent(nullptr) {}
};


class RRTStarPlanner {
public:
    Point2f start_pos_;
    Point2f target_pos_;
    vector<PolygonObstacle> obstacles_;
    float step_len_;
    float error_dis_;
    float radius_;
    Size2f config_size_;
    RRTStarNode* graph_start_;
    RRTStarNode* graph_end_;
    int MAX_GRAPH_SIZE = 10000;
    int CUR_GRAPH_SIZE = 0;
    bool plan_scuess_ = false;

    RRTStarPlanner(): graph_start_(nullptr), graph_end_(nullptr) {}
    RRTStarPlanner(Point2f start, Point2f target, vector<PolygonObstacle> obs, float step_len = 18, 
                   float radius = 10, float error_dis = 10,
                   Size2f config_size = Size2f(640, 480)): 
        start_pos_(start), 
        target_pos_(target), 
        obstacles_(obs),
        step_len_(step_len), 
        radius_(radius),
        error_dis_(error_dis),
        config_size_(config_size) {
        graph_start_ = new RRTStarNode(start);
        graph_end_ = new RRTStarNode(target);
        CUR_GRAPH_SIZE++;
    }
    ~RRTStarPlanner();
    RRTStarPlanner(const RRTStarPlanner&);
    RRTStarPlanner& operator=(const RRTStarPlanner&);


    bool Plan(Mat source_img, float int_delta = 0.01, bool plan_in_interior = false) {
        srand(time(NULL));
        plan_scuess_ = false;
        float div_width = RAND_MAX / config_size_.width,
              div_height = RAND_MAX / config_size_.height,
              min_cost = FLT_MAX;
        Point2f rand_pos = Point2f(0, 0);
        
        while (CUR_GRAPH_SIZE < MAX_GRAPH_SIZE) {
            rand_pos.x = rand() / div_width;
            rand_pos.y = rand() / div_height;

            if (CUR_GRAPH_SIZE % 50 == 0) {
                rand_pos = target_pos_;
                CUR_GRAPH_SIZE++;
            }

            RRTStarNode* nearest_node = NearestNode(rand_pos);
            if (normSqr(nearest_node->pos - rand_pos) < radius_ * radius_ / 100)
                continue;

            RRTStarNode* new_node = AddNewNode(nearest_node, rand_pos);
            if (PathObstacleFree(nearest_node, new_node)) {
                if (plan_in_interior)
                    if (MinDistanceToObstaclesVec(obstacles_, new_node->pos) < int_delta)
                        continue;

                Rewire(nearest_node, new_node, source_img);
                if (normSqr(new_node->pos - target_pos_) <= error_dis_* error_dis_) {
                    if (new_node->cost + cv::norm(graph_end_->pos - new_node->pos) < min_cost) {
                        graph_end_->parent = new_node;
                        min_cost = new_node->cost + cv::norm(graph_end_->pos - new_node->pos);
                    }
                    plan_scuess_ = true;
                }
                CUR_GRAPH_SIZE++;
                circle(source_img, new_node->pos, 3, Scalar(0,255,0), -1);
            }
            else    
                delete new_node;

            circle(source_img, start_pos_, 4, Scalar(255,0,0), -1);
            circle(source_img, target_pos_, 4, Scalar(255,0,0), -1);
            imshow("RRT* path planning", source_img);
            waitKey(1);
            if (CUR_GRAPH_SIZE == MAX_GRAPH_SIZE)
                destroyWindow("RRT* path planning");
            // cout << "-->CUR_GRAPH_SIZE " << CUR_GRAPH_SIZE << '\n';
        }
        if (!plan_scuess_)
            cout << "MAX_GRAPH_SIZE: " << MAX_GRAPH_SIZE << " is achieved with no path founded.\n";
        else
            cout << "Path found with cost: " << min_cost
                 << "\nOptimal cost: " << norm(graph_end_->pos - graph_start_->pos)
                 << '\n';   
        return plan_scuess_;
    }

    RRTStarNode* NearestNode(Point2f& rand_node) {
        RRTStarNode* res = graph_start_;
        queue<RRTStarNode*> level_pt;
        level_pt.push(graph_start_);
        float min_dis = normSqr(rand_node - graph_start_->pos), cur_dis;
        
        // bfs
        while (!level_pt.empty()) {
            int level_size = level_pt.size();
            for (int i = 0; i < level_size; i++) {
                RRTStarNode* cur_node = level_pt.front();
                level_pt.pop();
                for (auto pt : cur_node->adjacency_list) {
                    level_pt.push(pt);
                }
                cur_dis = normSqr(rand_node - cur_node->pos);
                if (cur_dis < min_dis) {
                    res = cur_node;
                    min_dis = cur_dis;
                }
            }
        }
        return res;
    }

    RRTStarNode* AddNewNode(RRTStarNode* nearest_node, Point2f& rand_pos) {
        Point2f direction = (rand_pos - nearest_node->pos) / cv::norm((rand_pos - nearest_node->pos));
        Point2f new_pos = nearest_node->pos + (direction * step_len_);
        RRTStarNode* new_node = new RRTStarNode(new_pos);
        // RRTStarNode* new_node = new RRTStarNode(rand_pos);
        return new_node;
    }

    void Rewire(RRTStarNode* nearest_node, RRTStarNode* new_node, Mat source_img) {
        float gamma_star = 800,
              gamma = gamma_star * sqrt(log(CUR_GRAPH_SIZE) * 3.32 / CUR_GRAPH_SIZE),
              radius_alg = min(gamma, step_len_);

        vector<RRTStarNode*> near_set;
        queue<RRTStarNode*> level_pt;
        level_pt.push(graph_start_);

        while (!level_pt.empty()) {
            int level_size = level_pt.size();
            for (int i = 0; i < level_size; i++) {
                RRTStarNode* cur_node = level_pt.front();
                level_pt.pop();
                for (auto pt : cur_node->adjacency_list)
                    level_pt.push(pt);
                if (normSqr(cur_node->pos - new_node->pos) <= radius_alg * radius_alg)
                    near_set.push_back(cur_node);
            }
        }

        // find minimal-cost path
        RRTStarNode* min_cost_node = nearest_node;
        float min_cost = UpdatePassageEncodingCost(nearest_node, new_node);
        for (auto near_node : near_set) {     
            float cur_cost = UpdatePassageEncodingCost(near_node, new_node);       
            if (cur_cost < min_cost && PathObstacleFree(near_node, new_node)) {
                min_cost_node = near_node;
                min_cost = cur_cost;
            }
        }

        new_node->parent = min_cost_node;
        new_node->cost = min_cost;
        new_node->min_passage_width = GetNewNodeMinPassageWidth(min_cost_node, new_node);
        min_cost_node->adjacency_list.push_back(new_node);
        line(source_img, min_cost_node->pos, new_node->pos, Scalar(0, 0, 200), 1.5);

        for (auto near_node : near_set) {
            float new_near_node_cost = UpdatePassageEncodingCost(new_node, near_node);
            if (new_near_node_cost < near_node->cost && PathObstacleFree(near_node, new_node)) {
                near_node->min_passage_width = GetNewNodeMinPassageWidth(new_node, near_node);
                RRTStarNode* near_node_parent = near_node->parent;
                for (int i = 0; i < near_node_parent->adjacency_list.size(); i++)
                    if (near_node_parent->adjacency_list[i] == near_node) {
                        near_node_parent->adjacency_list.erase(near_node_parent->adjacency_list.begin() + i);
                        break;
                    }
                near_node->parent = new_node;
                near_node->cost = new_near_node_cost;
                new_node->adjacency_list.push_back(near_node);
            }
        }
    }

    bool PathObstacleFree(RRTStarNode* near_node, RRTStarNode* new_node) {
        for (auto& obs : obstacles_)
            if (!ObstacleFree(obs, near_node->pos, new_node->pos))
                return false;
        return true;
    }

    float UpdatePassageEncodingCost(RRTStarNode* near_node, RRTStarNode* new_node) {
        float passage_width_passed = GetMinPassageWidthPassed(obstacles_, near_node->pos, new_node->pos);
        float res = 0;
        if (passage_width_passed < 0)
            res = near_node->cost + cv::norm(new_node->pos - near_node->pos) / near_node->min_passage_width;
        else 
            res = (near_node->cost * near_node->min_passage_width + cv::norm(new_node->pos - near_node->pos)) 
                        / min(near_node->min_passage_width, passage_width_passed); 
        return res;      
    }

    float GetNewNodeMinPassageWidth(RRTStarNode* parent_node, RRTStarNode* new_node) {
        float res = parent_node->min_passage_width;
        float min_passage_width_new_edge_passes = GetMinPassageWidthPassed(obstacles_, parent_node->pos, new_node->pos);
        if (min_passage_width_new_edge_passes < 0) 
            return res;
        else 
            return min(parent_node->min_passage_width, min_passage_width_new_edge_passes);
    }

    vector<RRTStarNode*> GetPath() {
        vector<RRTStarNode*> res;
        if (!plan_scuess_) {
            cout << "No valid path is available.\n";
            return res;
        }
        RRTStarNode* reverse_node = graph_end_;
        while (reverse_node) {
            res.push_back(reverse_node);
            reverse_node = reverse_node->parent;
        }
        reverse(res.begin(), res.end());
        return res;   
    }    
};


RRTStarPlanner::~RRTStarPlanner() {
    delete graph_start_;
    delete graph_end_;
}

RRTStarPlanner::RRTStarPlanner(const RRTStarPlanner& planner) {
    start_pos_ = planner.start_pos_;
    target_pos_ = planner.target_pos_;
    obstacles_ = planner.obstacles_;
    step_len_ = planner.step_len_;
    error_dis_ = planner.error_dis_;
    config_size_ = planner.config_size_;
    if (!graph_start_)
        graph_start_ = new RRTStarNode((*planner.graph_start_));
    else
        *graph_start_ = *(planner.graph_start_);
    if (!graph_end_)
        graph_end_ = new RRTStarNode(*(planner.graph_end_));
    else
        *graph_end_ = *(planner.graph_end_);
    MAX_GRAPH_SIZE = planner.MAX_GRAPH_SIZE;
    CUR_GRAPH_SIZE = planner.CUR_GRAPH_SIZE;
    plan_scuess_ = planner.plan_scuess_;        
}

RRTStarPlanner& RRTStarPlanner::operator=(const RRTStarPlanner& rhs) {
    start_pos_ = rhs.start_pos_;
    target_pos_ = rhs.target_pos_;
    obstacles_ = rhs.obstacles_;
    step_len_ = rhs.step_len_;
    error_dis_ = rhs.error_dis_;
    config_size_ = rhs.config_size_;
    if (!graph_start_)
        graph_start_ = new RRTStarNode((*rhs.graph_start_));
    else
        *graph_start_ = *(rhs.graph_start_);
    if (!graph_end_)
        graph_end_ = new RRTStarNode(*(rhs.graph_end_));
    else
        *graph_end_ = *(rhs.graph_end_);
    MAX_GRAPH_SIZE = rhs.MAX_GRAPH_SIZE;
    CUR_GRAPH_SIZE = rhs.CUR_GRAPH_SIZE;
    plan_scuess_ = rhs.plan_scuess_;
}

#endif