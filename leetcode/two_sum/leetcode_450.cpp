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
    TreeNode* deleteNode(TreeNode* root, int key) {
        if (root == nullptr)
            return nullptr;
        
        if (root->val == key) {
            if (root->left == nullptr)
                return root->right;
            else if (root->right == nullptr)
                return root->left;
            
            TreeNode* mostRightofLeft = root->left;
            while (mostRightofLeft->right != nullptr)
                mostRightofLeft = mostRightofLeft->right;
            mostRightofLeft->right = root->right;
            return root->left;
        }
        else if (root->val > key) 
            root->left = deleteNode(root->left, key);
        else 
            root->right = deleteNode(root->right, key);
        return root;
    }
};
