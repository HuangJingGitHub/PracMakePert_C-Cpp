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
    vector<int> valVec;
    bool isSymmetric(TreeNode* root) {
        if (root == NULL)
            return true;
            
        return check(root->left, root->right);       
    }

    bool check(TreeNode* node1, TreeNode* node2)
    {
        if (node1 == NULL && node2 != NULL)
            return false;
        else if (node1 != NULL && node2 == NULL)
            return false;
        else if (node1 == NULL && node2 == NULL)
            return true;
        else if (node1->val != node2->val)
            return false;
        return check(node1->left, node2->right) && check(node1->right, node2->left);
    }
};
