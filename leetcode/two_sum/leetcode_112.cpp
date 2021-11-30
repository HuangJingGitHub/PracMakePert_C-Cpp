/**
 * Definition for a binary tree node.
 * struct TreeNode {
 *     int val;
 *     TreeNode *left;
 *     TreeNode *right;
 *     TreeNode(int x) : val(x), left(NULL), right(NULL) {}
 * };
 */
class Solution {
public:
    bool hasPathSum(TreeNode* root, int sum) {
        if (root == NULL)
            return false;
        else if (!root->left && !root->right)
            return sum == root->val;

        // else if (root->left && !root->right)  // The commented part is redundant. Keeping the last case is sufficient and effective.
        //    return hasPathSum(root->left, sum - root->val);
        // else if (!root->left && root->right)
        //    return hasPathSum(root->right, sum - root->val);
        
        else
            return hasPathSum(root->left, sum - root->val) || hasPathSum(root->right, sum - root->val);
    }
};
