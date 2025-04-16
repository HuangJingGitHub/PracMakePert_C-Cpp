#ifndef KD_TREE_INCLUDED
#define KD_TREE_INCLUDED

#include <list>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

//using namespace cv;
//using namespace std;

float SquaredNorm(const cv::Point2f& pt) {
    return pt.x * pt.x + pt.y * pt.y;
}

struct PathNode {
    int id = 0;
    cv::Point2f pos;
    float cost = 0;
    float len = 0;
    float min_passage_width = 10000;
    // The passage width passed by the edge parent node---current node
    float cur_passage_width = -1;
    std::list<float> sorted_passage_list;
    // std::set<int> passage_idx_set;
    PathNode* parent;
    std::list<PathNode*> children;
    PathNode* left;
    PathNode* right;
    PathNode(): pos(cv::Point2f(0, 0)), parent(nullptr), left(nullptr), right(nullptr) {}
    PathNode(cv::Point2f initPos): pos(initPos), parent(nullptr), left(nullptr), right(nullptr) {}
};

class kdTree{
private:
    const int kDimension_k_ = 2;
public:
    PathNode* kd_tree_root_;

    kdTree(): kd_tree_root_(nullptr) {}
    kdTree(PathNode* root_node): kd_tree_root_(root_node) {};
    ~kdTree();
    kdTree(const kdTree&);
    kdTree& operator=(const kdTree&);

    void AddWithRoot(PathNode* root, PathNode* new_node, int depth) {
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

    void Add(PathNode* new_node) {
        if (new_node == nullptr)
            return;

        if (kd_tree_root_ == nullptr)
            kd_tree_root_ = new_node;
        else
            AddWithRoot(kd_tree_root_, new_node, 0);
    }

    PathNode* GetCloserInTwo(PathNode* target, PathNode* candidate_1, PathNode* candidate_2) {
        if (candidate_1 == nullptr)
            return candidate_2;
        if (candidate_2 == nullptr)
            return candidate_1;

        if (SquaredNorm(target->pos - candidate_1->pos) <= SquaredNorm(target->pos - candidate_2->pos))
            return candidate_1;
        return candidate_2;
    }

    PathNode* FindNearestNodeWithRoot(PathNode* root, PathNode* target, int depth) {
        if (root == nullptr)
            return nullptr;
        
        PathNode *next_subtree = nullptr, *other_subtree = nullptr;
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
        PathNode *temp_res = FindNearestNodeWithRoot(next_subtree, target, depth + 1),
                    *cur_best = GetCloserInTwo(target, temp_res, root);
        float cur_dist_square = SquaredNorm(target->pos - cur_best->pos), dist_to_boundary_sqr, dist_to_boundary;
        if (depth % kDimension_k_ == 0) 
            dist_to_boundary = target->pos.x - root->pos.x;
        else
            dist_to_boundary = target->pos.y - root->pos.y;
        dist_to_boundary_sqr = dist_to_boundary * dist_to_boundary;
        
        if (cur_dist_square >= dist_to_boundary_sqr) {
            temp_res = FindNearestNodeWithRoot(other_subtree, target, depth + 1);
            cur_best = GetCloserInTwo(target, temp_res, cur_best);
        }
        return cur_best;
    }

    PathNode* FindNearestNode(PathNode* target) {
        return FindNearestNodeWithRoot(kd_tree_root_, target, 0);
    } 
   
    PathNode* FindNearestNode(const cv::Point2f& target_pos) {
        PathNode* target_node = new PathNode(target_pos);
        PathNode* res = FindNearestNodeWithRoot(kd_tree_root_, target_node, 0);
        delete target_node;
        return res;
    }

    void RangeSearchWithRoot(PathNode* root, std::vector<PathNode*>& res_pt_vec, 
                            const float& x_min, const float& x_max, 
                            const float& y_min, const float& y_max, int depth) {
        if (root == nullptr)
            return;

        if (root->pos.x >= x_min && root->pos.x <= x_max && root->pos.y >= y_min && root->pos.y <= y_max)
            res_pt_vec.push_back(root);
        
        if (depth % kDimension_k_ == 0) {
            if (root->pos.x < x_min)
                RangeSearchWithRoot(root->right, res_pt_vec, x_min, x_max, y_min, y_max, depth + 1);
            else if (root->pos.x > x_max)
                RangeSearchWithRoot(root->left, res_pt_vec, x_min, x_max, y_min, y_max, depth + 1);
            else {
                RangeSearchWithRoot(root->left, res_pt_vec, x_min, x_max, y_min, y_max, depth + 1);
                RangeSearchWithRoot(root->right, res_pt_vec, x_min, x_max, y_min, y_max, depth + 1);
            }   
        }
        else {
            if (root->pos.y < y_min)
                RangeSearchWithRoot(root->right, res_pt_vec, x_min, x_max, y_min, y_max, depth + 1);
            else if (root->pos.y > y_max)
                RangeSearchWithRoot(root->left, res_pt_vec, x_min, x_max, y_min, y_max, depth + 1);
            else {
                RangeSearchWithRoot(root->left, res_pt_vec, x_min, x_max, y_min, y_max, depth + 1);
                RangeSearchWithRoot(root->right, res_pt_vec, x_min, x_max, y_min, y_max, depth + 1);
            }              
        }         
    }

    std::vector<PathNode*> RanageSearch(const float& x_min, const float& x_max, const float& y_min, float& y_max) {
        std::vector<PathNode*> res;
        if (x_min > x_max || y_min > y_max) {
            throw std::invalid_argument("Invalid range in range search. " + std::string(__func__));             
        }
        RangeSearchWithRoot(kd_tree_root_, res, x_min, x_max, y_min, y_max, 0);
        return res;
    }

    void deleteTree(PathNode* root) {
        if (root == nullptr)
            return;
        
        deleteTree(root->left);
        deleteTree(root->right);
        delete root;
    }
};

kdTree::~kdTree() {
    kdTree::deleteTree(kd_tree_root_);
} 

kdTree::kdTree(const kdTree& copied_tree) {
    kd_tree_root_ = copied_tree.kd_tree_root_;
}

kdTree& kdTree::operator=(const kdTree& rhs) {
    kd_tree_root_ = rhs.kd_tree_root_;
    return *this;
}
#endif
