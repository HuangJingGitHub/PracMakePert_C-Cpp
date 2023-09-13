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
    vector<int> largestValues(TreeNode* root) {
        if (root == nullptr)
            return {};
            
        vector<int> res;
        queue<TreeNode*> level;
        level.push(root);

        while (level.empty() == false) {
            int level_size = level.size();
            int largest = level.front()->val;
            for (int i = 0; i < level_size; i++) {
                TreeNode* cur_node = level.front();
                level.pop();
                largest = max(largest, cur_node->val);
                if (cur_node->left != nullptr)
                    level.push(cur_node->left);
                if (cur_node->right != nullptr)
                    level.push(cur_node->right);
            }
            res.push_back(largest);
        }
        return res;
    }
};
