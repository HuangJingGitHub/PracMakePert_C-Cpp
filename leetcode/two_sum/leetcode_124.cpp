// recursion, see the discussion
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
    int res = INT_MIN;
    int maxPathSum(TreeNode* root) {
        getMax(root);
        return res;
    }

    int getMax(TreeNode* r){
        if (r == NULL)
            return 0;
        int left = max(0, getMax(r->left));
        int right = max(0, getMax(r->right));
        res = max(res, r->val + left + right);
        return max(left, right) + r->val;
    }
};
