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

struct Node {
    Point2f pos;
    Node* parent;
    vector<Node*> adjacency_list;
    Node(Point2f initPos): pos(initPos), parent(nullptr) {}
};

class RRT_Planner {
public:
    Point2f start_pos_;
    Point2f target_pos_;
    float step_len_;
    float error_dis_;
    Size2f config_size_;
    Node* search_graph_;
    int MAX_TREE_SIZE_ = 300;
    int CUR_TREE_SIZE_ = 0;
    bool plan_scuess_ = false;

    RRT_Planner() {}
    RRT_Planner(Point2f start, Point2f target, float step_len = 1, float error_dis = 1,
                Size2f config_size = Size2f(640, 480)): 
        start_pos_(start), target_pos_(end), step_len_(step_len), error_dis_(error_dis)
        config_size_(config_size) {
        search_graph_ = new Node(start);
        CUR_TREE_SIZE_++;
    }

    ~RRT_Planner() {
        delete search_graph_;
    }

    bool Plan() {
        srand(time(NULL));
        float div_width = RAND_MAX / config_size_.width,
              div_height = RAND_MAX / config_size_.height;
        Point2f rand_pos = Point2f(0, 0);

        while (CUR_TREE_SIZE_ < MAX_TREE_SIZE_) {
            rand_pos.x = rand() / div_width;
            rand_pos.y = rand() / div_height;
            
            Node* nearest_node = NearestNode(rand_pos)
            Node* new_node = AddNewNode(nearest_node, rand_pos);

        }
    }

    Node* NearestNode(Point2f& rand_node) {
        Node* res = search_graph_;
        queue<Node*> level_pt;
        folat min_dis = norm(rand_node - search_graph_->pos), cur_dis;
        for (auto pt : search_graph_->adjacency_list)
            level_pt.push(pt);
        // bfs
        while (!level_pt.empty()) {
            int level_size = level_pt.size();
            for (int i = 0; i < level_size; i++) {
                Node* cur_node = level_pt.front();
                level_pt.pop();
                for (auto pt : cur_node->adjacency_list)
                    level_pt.push(pt);
                cur_dis = norm(rand_node - cur_node->pos);
                if (cur_dis < min_dis)
                    res = cur_node;
            }
        }
        return res;
    }

    Node* AddNewNode(Node* nearest_node, Point2f& rand_pos) {
        Point2f direction = (rand_pos - nearest_node->pos) / norm((rand_pos - nearest_node->pos));
        Point2f new_pos = nearest_node->pos + direction * step_len_;
        Node* new_node = new Node(new_pos);   // exist time ?
        new_node->parent = nearest_node;
        nearest_node->adjacency_list.push_back(new_node);
        return new_node;
    }
};
