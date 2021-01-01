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

struct RRT_Node {
    Point2f pos;
    RRT_Node* parent;
    vector<RRT_Node*> adjacency_list;
    RRT_Node(): pos(Point2f(0, 0)), parent(nullptr) {}
    RRT_Node(Point2f initPos): pos(initPos), parent(nullptr) {}
};

class RRT_Planner {
public:
    Point2f start_pos_;
    Point2f target_pos_;
    float step_len_;
    float error_dis_;
    Size2f config_size_;
    RRT_Node* search_graph_;
    RRT_Node* graph_end_;
    int MAX_GRAPH_SIZE = 300;
    int CUR_GRAPH_SIZE = 0;
    bool plan_scuess_ = false;

    RRT_Planner(): search_graph_(nullptr), graph_end_(nullptr) {}
    RRT_Planner(Point2f start, Point2f target, float step_len = 2, float error_dis = 2,
                Size2f config_size = Size2f(640, 480)): 
        start_pos_(start), 
        target_pos_(target), 
        step_len_(step_len), 
        error_dis_(error_dis),
        config_size_(config_size) {
        search_graph_ = new RRT_Node(start);
        graph_end_ = new RRT_Node();
        CUR_GRAPH_SIZE++;
    }

    ~RRT_Planner() {
        delete search_graph_;
        delete graph_end_;
    }

    bool Plan() {
        srand(time(NULL));
        plan_scuess_ = false;
        float div_width = RAND_MAX / config_size_.width,
              div_height = RAND_MAX / config_size_.height;
        Point2f rand_pos = Point2f(0, 0);
        
        while (CUR_GRAPH_SIZE < MAX_GRAPH_SIZE) {
            rand_pos.x = rand() / div_width;
            rand_pos.y = rand() / div_height;

            cout << "CUR_GRAPH_SIZE " << CUR_GRAPH_SIZE << '\n';
            RRT_Node* nearest_node = NearestNode(rand_pos);
            RRT_Node* new_node = AddNewNode(nearest_node, rand_pos);
            if (norm(new_node->pos - target_pos_) <= error_dis_) {
                graph_end_= new_node;
                plan_scuess_ = true;
                return true;
            }   
            CUR_GRAPH_SIZE++;
        }
        cout << "MAX_GRAPH_SIZE: " << MAX_GRAPH_SIZE << " is achieved with no path founded.\n";
        return false;
    }

    RRT_Node* NearestNode(Point2f& rand_node) {
        RRT_Node* res = search_graph_;
        queue<RRT_Node*> level_pt;
        float min_dis = norm(rand_node - search_graph_->pos), cur_dis;
        for (auto pt : search_graph_->adjacency_list)
            level_pt.push(pt);
        
        // bfs
        cout << "search_graph_: " << search_graph_ << '\n';
        while (!level_pt.empty()) {
            queue<RRT_Node*> tempQueue = level_pt;
            cout << "node_queue:\n";
            while(!tempQueue.empty()) {
                if (!tempQueue.front()->adjacency_list.empty())
                cout << tempQueue.front() << " VS " << tempQueue.front()->adjacency_list[0]
                    << '\n';
                tempQueue.pop();
            }

            int level_size = level_pt.size();
            for (int i = 0; i < level_size; i++) {
                RRT_Node* cur_node = level_pt.front();
                level_pt.pop();

                string s;
                cin >> s;

                cout << "cur_node " << cur_node->pos << '\n';
                cout << "adjacency_list: \n";
                for (auto pt : cur_node->adjacency_list) {
                    level_pt.push(pt);
                    cout << pt << '\n';
                }
                cur_dis = norm(rand_node - cur_node->pos);
                cout << "cur_dis " << cur_dis << '\n';
                if (cur_dis < min_dis)
                    res = cur_node;
            }
        }
        return res;
    }

    RRT_Node* AddNewNode(RRT_Node* nearest_node, Point2f& rand_pos) {
        Point2f direction = (rand_pos - nearest_node->pos) / norm((rand_pos - nearest_node->pos));
        Point2f new_pos = nearest_node->pos + direction * step_len_;
        static RRT_Node new_node_obj = RRT_Node(new_pos);   // exist time ?
        RRT_Node* new_node = &new_node_obj;
        new_node->parent = nearest_node;
        nearest_node->adjacency_list.push_back(new_node);
        return new_node;
    }

    vector<RRT_Node*> GetPath() {
        vector<RRT_Node*> res;
        if (!plan_scuess_) {
            cout << "No valid path available.\n";
            return res;
        }
        RRT_Node* reverse_node = graph_end_;
        while (reverse_node) {
            res.push_back(reverse_node);
            reverse_node = reverse_node->parent;
        }
        return res;   
    }    
};
