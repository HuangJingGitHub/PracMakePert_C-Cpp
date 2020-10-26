// It is a rather theoretical problem. For practice, refer to the favorited solution article.
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
    int res = 0, s = 1;
    int navigation(TreeNode* root) {
        int left = DFS(root->left), right = DFS(root->right);
        return res + ((left && right) ? 0 : s);
    }

    int DFS(TreeNode* root){
        if (!root)
            return 0;
        
        int left = DFS(root->left), right = DFS(root->right);
        if (root->left && root->right){
            res += !left && !right;
            s = !(left && right);
            return 1;
        }
        return left || right;
    }
};
