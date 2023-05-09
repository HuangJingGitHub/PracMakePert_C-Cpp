#ifndef KD_TREE_INCLUDED
#define KD_TREE_INCLUDED

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

float normSqr(Point2f pt) {
    return pt.x * pt.x + pt.y * pt.y;
}

struct RRTStarNode {
    Point2f pos;
    float cost;
    float min_passage_width = 500;
    RRTStarNode* parent;
    RRTStarNode* left;
    RRTStarNode* right;
    vector<RRTStarNode*> adjacency_list;
    RRTStarNode(): pos(Point2f(0, 0)), cost(0), parent(nullptr), left(nullptr), right(nullptr) {}
    RRTStarNode(Point2f initPos): pos(initPos), cost(0), parent(nullptr), left(nullptr), right(nullptr) {}
};

class kdTree{
private:
    const int kDimension_k_ = 2;
public:
    RRTStarNode* kd_tree_root_;

    kdTree(): kd_tree_root_(nullptr) {}
    kdTree(RRTStarNode* root_node): kd_tree_root_(root_node) {};

    void AddWithRoot(RRTStarNode* root, RRTStarNode* new_node, int depth) {
        if (depth % kDimension_k_ == 0) {
            if (new_node->pos.x <= root->pos.x) {
                if (root->left == nullptr) 
                    root->left = new_node;
                else 
                    AddWithRoot(root->left, new_node, depth + 1);
            }
            else {
                if (root->right == nullptr) 
                    root->right = new_node;
                else 
                    AddWithRoot(root->right, new_node, depth + 1);
            }
        }
        else {
            if (new_node->pos.y <= root->pos.y) {
                if (root->left == nullptr) 
                    root->left = new_node;
                else 
                    AddWithRoot(root->left, new_node, depth + 1);
            }
            else {
                if (root->right == nullptr) 
                    root->right = new_node;
                else
                    AddWithRoot(root->right, new_node, depth + 1);
            }
        }
    }

    void Add(RRTStarNode* new_node) {
        if (new_node == nullptr)
            return;

        if (kd_tree_root_ == nullptr)
            kd_tree_root_ = new_node;
        else
            AddWithRoot(kd_tree_root_, new_node, 0);
    }

    RRTStarNode* GetCloestInTwo(RRTStarNode* target, RRTStarNode* candidate_1, RRTStarNode* candidate_2) {
        if (normSqr(target->pos - candidate_1->pos) <= normSqr(target->pos - candidate_2->pos))
            return candidate_1;
        return candidate_2;
    }

    RRTStarNode* FindNearestNodeWithRoot(RRTStarNode* root, RRTStarNode* target, int depth) {
        if (root == nullptr)
            return nullptr;
        
        RRTStarNode *next_subtree = nullptr, *other_subtree = nullptr;
        if (depth % kDimension_k_ == 0) {
            if (target->pos.x <= root->pos.x) {
                next_subtree = root->left;
                other_subtree = root->right;
            }
            else {
                next_subtree = root->right;
                other_subtree = root->left;
            }
        }
        else {
            if (target->pos.y <= root->pos.y) {
                next_subtree = root->left;
                other_subtree = root->right;
            }
            else {
                next_subtree = root->right;
                other_subtree = root->left;
            }  
        }

        RRTStarNode *temp_res = FindNearestNodeWithRoot(next_subtree, target, depth + 1),
                    *cur_best = nullptr;
        if (normSqr(target->pos - temp_res->pos) <= normSqr(target->pos - root->pos))
            cur_best = temp_res;
        else
            cur_best = root;

        float cur_dist_square = normSqr(target->pos - cur_best->pos), dist_to_boundary_square, dist_to_boundary;
        if (depth % kDimension_k_ == 0) 
            dist_to_boundary = target->pos.x - root->pos.x;
        else
            dist_to_boundary = target->pos.y - root->pos.y;
        dist_to_boundary_square = dist_to_boundary * dist_to_boundary;
        
    }

    RRTStarNode* FindNearestNode(RRTStarNode* target) {
        return FindNearestNodeWithRoot(kd_tree_root_, target, 0);
   } 
};

#endif