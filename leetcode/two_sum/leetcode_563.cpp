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
    int res = 0;
    int findTilt(TreeNode* root) {
        getNodeSum(root);
        return res;
    }

    int getNodeSum(TreeNode* root) {
        if (root == nullptr)
            return 0;
        int left_sum = getNodeSum(root->left),
            right_sum = getNodeSum(root->right);
        res += abs(left_sum - right_sum);
        return left_sum + right_sum + root->val;
    }
};
