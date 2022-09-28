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
    long long preNodeVal = LLONG_MIN;
    bool isValidBST(TreeNode* root) {
        if (root == nullptr)
            return true;
        
        if (isValidBST(root->left) == false) 
            return false;
        if (root->val <= preNodeVal)   // inorder check
            return false;
        preNodeVal = root->val;

        return isValidBST(root->right);
    }
};
