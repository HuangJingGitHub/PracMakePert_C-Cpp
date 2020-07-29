// Checking if a tree is balanced is fundamental, check the reference solutions. 
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
    bool isBalanced(TreeNode* root) {
        return checkHeight(root) != -1;
    }
    
    int checkHeight(TreeNode* root)
    {
        if (root == NULL)
            return 0;
        int leftHeight = checkHeight(root->left);
        if (leftHeight == -1)
            return -1;

        int rightHeight = checkHeight(root->right);
        if (rightHeight == -1)
            return -1;
        return abs(leftHeight - rightHeight) < 2 ? (max(leftHeight, rightHeight)+1) : -1;   // Note the height of a tree = max(leftHeight, rightHeight).
    }
};
