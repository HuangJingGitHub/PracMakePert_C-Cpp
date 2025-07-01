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
    string res = "";

    string tree2str(TreeNode* root) {
        dfs(root);
        return res;
    }

    void dfs(TreeNode* node) {
        if (node == nullptr)
            return;

        res += to_string(node->val);
        if (node->left != nullptr || node->right != nullptr) {
            res += "(";
            dfs(node->left);
            res += ")";
        }
        if (node->right != nullptr) {
            res += "(";
            dfs(node->right);
            res += ")";
        }
    }
};
