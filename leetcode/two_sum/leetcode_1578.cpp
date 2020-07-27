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
    bool isSymmetric(TreeNode* root) {
        if (root == NULL)
            return true;
        
        return checkSymmetric(root->left, root->right);

    }

    bool checkSymmetric(TreeNode* left, TreeNode* right)
    {
        if (left == NULL && right == NULL)
            return true;
        else if ((!left && right) || (left && !right) || left->val != right->val)
            return false;
        else
             return  checkSymmetric(left->left, right->right) && checkSymmetric(left->right, right->left);
    }
};
