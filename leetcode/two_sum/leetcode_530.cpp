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
    int getMinimumDifference(TreeNode* root) {
        int pre = -1, res = INT_MAX;
        inOrderTraverse(root, pre, res);
        return res;
    }

    void inOrderTraverse(TreeNode* root, int& pre, int& res) {
        if (root == nullptr)
            return;
            
        inOrderTraverse(root->left, pre, res);
        if (pre != -1)
            res = min(res, root->val - pre);
        pre = root->val;
        inOrderTraverse(root->right, pre, res);
    }
};
