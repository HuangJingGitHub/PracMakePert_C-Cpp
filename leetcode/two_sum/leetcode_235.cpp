class Solution {
public:
    TreeNode* lowestCommonAncestor(TreeNode* root, TreeNode* p, TreeNode* q) {
        if ((root->val - p->val) * (root->val - q->val) <= 0)
            return root;
        else if (root->val > p->val)
            return lowestCommonAncestor(root->left, p, q);
        else 
            return lowestCommonAncestor(root->right, p, q);
    }
};
