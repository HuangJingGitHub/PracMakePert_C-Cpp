/**
 * Definition for a binary tree node.
 * struct TreeNode {
 *     int val;
 *     TreeNode *left;
 *     TreeNode *right;
 *     TreeNode() : val(0), left(nullptr), right(nullptr) {}
 *     TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}
 *     TreeNode(int x, TreeNode *left, TreeNode *right) : val(x), left(left), right(right) {}
 * };
 */
class Solution {
public:
    TreeNode* addOneRow(TreeNode* root, int val, int depth) {
        if (depth == 1) {
            TreeNode* res = new TreeNode(val, root, nullptr);
            return res;
        }

        queue<TreeNode*> node_level;
        node_level.push(root);
        int cur_depth = 1;
        while (cur_depth < depth - 1) {
            int level_size = node_level.size();
            while (level_size > 0) {
                TreeNode* node = node_level.front();
                node_level.pop();
                if (node->left) {
                    node_level.push(node->left);
                }
                if (node->right) {
                    node_level.push(node->right);
                }
                level_size--;
            }
            cur_depth++;
        }

        while (!node_level.empty()) {
            TreeNode* node = node_level.front();
            node_level.pop();
            
            TreeNode* new_left = new TreeNode(val, node->left, nullptr);
            TreeNode* new_right = new TreeNode(val, nullptr, node->right);
            node->left = new_left;
            node->right = new_right;
        }
        return root;
    }
};
