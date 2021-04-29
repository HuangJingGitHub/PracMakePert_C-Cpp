class Solution {
public:
    TreeNode* invertTree(TreeNode* root) {
        if (!root)
            return root;
        else {
            TreeNode* leftPt = root->left;
            root->left = root->right;
            root->right = leftPt;
            invertTree(root->left);
            invertTree(root->right);
        }
        return root;
    }
};
