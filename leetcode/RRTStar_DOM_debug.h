#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <queue>
#include <algorithm>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

using namespace cv;
using namespace std;

struct RRTStarNode {
    Point2f pos;
    float cost;
    RRTStarNode* parent;
    vector<RRTStarNode*> adjacency_list;
    RRTStarNode(): pos(Point2f(0, 0)), cost(0), parent(nullptr) {}
    RRTStarNode(Point2f initPos): pos(initPos), cost(0), parent(nullptr) {}
};

class RRTStarNodeComparator {
    Point2f reference_pos_;
public:
    RRTStarNodeComparator(Point2f pos): reference_pos_(pos) {}

    bool operator()(const RRTStarNode* node1, const RRTStarNode* node2) {
        return norm(node1->pos - reference_pos_) > norm(node2->pos - reference_pos_);
    }
};


class RRTStarPlanner {
public:
    Point2f start_pos_;
    Point2f target_pos_;
    float step_len_;
    float error_dis_;
    float radius_;
    Size2f config_size_;
    RRTStarNode* graph_start_;
    RRTStarNode* graph_end_;
    int MAX_GRAPH_SIZE = 5000;
    int CUR_GRAPH_SIZE = 0;
    bool plan_scuess_ = false;

    RRTStarPlanner(): graph_start_(nullptr), graph_end_(nullptr) {}
    RRTStarPlanner(Point2f start, Point2f target, float step_len = 10, float error_dis = 10, float radius = 40,
                Size2f config_size = Size2f(640, 480)): 
        start_pos_(start), 
        target_pos_(target), 
        step_len_(step_len), 
        error_dis_(error_dis),
        radius_(radius),
        config_size_(config_size) {
        graph_start_ = new RRTStarNode(start);
        graph_end_ = new RRTStarNode(target);
        CUR_GRAPH_SIZE++;
    }
    ~RRTStarPlanner();
    RRTStarPlanner(const RRTStarPlanner&);
    RRTStarPlanner& operator=(const RRTStarPlanner&);

    bool Plan(Mat source_img) {
        srand(time(NULL));
        plan_scuess_ = false;
        float div_width = RAND_MAX / config_size_.width,
              div_height = RAND_MAX / config_size_.height,
              min_cost = 1e8;
        Point2f rand_pos = Point2f(0, 0);
        
        while (CUR_GRAPH_SIZE < MAX_GRAPH_SIZE) {
            rand_pos.x = rand() / div_width;
            rand_pos.y = rand() / div_height;

            cout << "*****-->CUR_GRAPH_SIZE " << CUR_GRAPH_SIZE << '\n';
            RRTStarNode* nearest_node = NearestNode(rand_pos);
            RRTStarNode* new_node = AddNewNode(nearest_node, rand_pos);
            cout << "Added new node address: " << new_node << " --> pos " << new_node->pos << '\n';
            rewire(nearest_node, new_node);
            if (norm(new_node->pos - target_pos_) <= error_dis_) {
                if (new_node->cost + norm(graph_end_->pos - new_node->pos) < min_cost){
                    graph_end_->parent = new_node;
                    min_cost = new_node->cost + norm(graph_end_->pos - new_node->pos);
                }
                plan_scuess_ = true;
            }
            circle(source_img, start_pos_, 5, Scalar(0,0,255), -1);
            circle(source_img, target_pos_, 5, Scalar(0,0,255), -1);
            circle(source_img, new_node->pos, 3, Scalar(0,255,0), -1);
            imshow("RRT path planning", source_img);
            waitKey(10);
            CUR_GRAPH_SIZE++;
        }
        if (!plan_scuess_)
            cout << "MAX_GRAPH_SIZE: " << MAX_GRAPH_SIZE << " is achieved with no path founded.\n";
        else
            cout << "Path found with cost: " << min_cost
                << "\nOptimal cost: " << norm(graph_end_->pos - graph_start_->pos)
                << '\n';   
        return plan_scuess_;
        // return false;
    }

    RRTStarNode* NearestNode(Point2f& rand_node) {
        RRTStarNode* res = graph_start_;
        queue<RRTStarNode*> level_pt;
        float min_dis = norm(rand_node - graph_start_->pos), cur_dis;
        for (auto pt : graph_start_->adjacency_list) {
            level_pt.push(pt);
        }
        
        // bfs
        while (!level_pt.empty()) {
            int level_size = level_pt.size();
            for (int i = 0; i < level_size; i++) {
                RRTStarNode* cur_node = level_pt.front();
                level_pt.pop();
                for (auto pt : cur_node->adjacency_list) {
                    level_pt.push(pt);
                }
                cur_dis = norm(rand_node - cur_node->pos);
                if (cur_dis < min_dis)
                    res = cur_node;
            }
        }
        return res;
    }

    RRTStarNode* AddNewNode(RRTStarNode* nearest_node, Point2f& rand_pos) {
        Point2f direction = (rand_pos - nearest_node->pos) / norm((rand_pos - nearest_node->pos));
        Point2f new_pos = nearest_node->pos + direction * step_len_;
        RRTStarNode* new_node = new RRTStarNode(new_pos);
        // new_node->parent = nearest_node;
        // nearest_node->adjacency_list.push_back(new_node);
        return new_node;
    }

    void rewire(RRTStarNode* nearest_node, RRTStarNode* new_node) {
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
                if (norm(cur_node->pos - new_node->pos) <= radius_)
                    near_set.push_back(cur_node);
            }
        }

        // find minimal cost path
        RRTStarNode* cost_min_node = nearest_node;
        float min_cost = nearest_node->cost + norm(new_node->pos - nearest_node->pos);
        for (auto near_node : near_set) {
            float cur_cost = near_node->cost + norm(new_node->pos - near_node->pos);
            if (cur_cost < min_cost) {
                cost_min_node = near_node;
                min_cost = cur_cost;
            }
        }

        new_node->parent = cost_min_node;
        new_node->cost = min_cost;
        cost_min_node->adjacency_list.push_back(new_node);

        for (auto near_node : near_set) {
            if (new_node->cost + norm(new_node->pos - near_node->pos) < near_node->cost) {
                RRTStarNode* near_node_parent = near_node->parent;
                for (int i = 0; i < near_node_parent->adjacency_list.size(); i++)
                    if (near_node_parent->adjacency_list[i] == near_node) {
                        near_node_parent->adjacency_list.erase(near_node_parent->adjacency_list.begin() + i);
                        break;
                    }
                near_node->parent = new_node;
                near_node->cost = new_node->cost + norm(new_node->pos - near_node->pos);
                new_node->adjacency_list.push_back(near_node);
            }
        }

    }

    vector<RRTStarNode*> GetPath() {
        vector<RRTStarNode*> res;
        if (!plan_scuess_) {
            cout << "No valid path available.\n";
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


/* class NodeMinHeap {
    priority_queue<RRTStarNode*, vector<RRTStarNode*>, RRTStarNodeComparator> node_min_heap_;
    int heap_size_ = 0;
    int size_ = 0;
    float cur_min_dis_ = 1e8;
    float cur_max_dis_ = 0;

    NodeMinHeap() {}
    NodeMinHeap(int size = 20): heap_size_(size) {}

public: 
    bool empty() {
        return node_min_heap_.empty();
    }

    int size() {
        return node_min_heap_.size();
    }

    int maxsize() {
        return heap_size_;
    }

    void push(RRTStarNode* node, Point2f& rand_pos) {
        if (size_ < heap_size_) {
            node_min_heap_.push(node);

            float cur_dis = norm(rand_pos - node->pos);
            cur_min_dis_ = min(cur_dis, cur_min_dis_);
            cur_max_dis_ = max(cur_dis, cur_max_dis_);
            size_++;
            return;
        }
        
        float cur_dis = norm(rand_pos - node->pos),
        if (cur_dis > cur_max_dis_)
            return; 
    }
} */