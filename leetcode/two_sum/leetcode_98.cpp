/**
 * Definition for a binary tree node.
 * struct TreeNode {
 *     int val;
 *     TreeNode *left;
 *     TreeNode *right;
 *     TreeNode(int x) : val(x), left(NULL), right(NULL) {}
 * };
 */
 // Inorder traversal
class Solution {
public:
    long long top = LLONG_MIN;    // smaller than int minimum INT_MIN
    bool isValid = true;
    bool isValidBST(TreeNode* root) {
        if (root) {
            isValidBST(root->left);
            if (root->val <= top)
                isValid = false;
            else {
                top = root->val;
                isValidBST(root->right);
            }
        }
        return isValid;
    }
};
