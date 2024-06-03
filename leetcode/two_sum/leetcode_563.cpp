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
    int findTilt(TreeNode* root) {
        int res = 0;
        if (root == nullptr)
            return 0;

        int left_sum = findTilt(root->left),
            right_sum = findTilt(root->right);
        root->val += left_sum + right_sum;
        int left_val = root->left == nullptr ? 0 : root->left->val;
        int right_val = root->right == nullptr ? 0 : root->right->val;
        res += abs(left_val - right_val);
        return res;
    }
};
