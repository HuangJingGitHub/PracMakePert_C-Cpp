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
    int longestUnivaluePath(TreeNode* root) {
        int res = 0;
        dfs(root, res);
        return res;
    }

    // return the longest univalue path length comtaining root
    int dfs(TreeNode* root, int& res) {
        if (root == nullptr)
            return -1;
        int left_len = dfs(root->left, res) + 1,
            right_len = dfs(root->right, res) + 1;
        if (root->left && root->left->val != root->val)
            left_len = 0;
        if (root->right && root->right->val != root->val)
            right_len = 0;
        res = max(res, left_len + right_len);
        return max(left_len, right_len);
    }

};
