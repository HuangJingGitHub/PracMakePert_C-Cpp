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
    TreeNode* first = NULL;
    TreeNode* second = NULL;
    TreeNode* pre = NULL;
    
    void recoverTree(TreeNode* root) {
        inorderTraversal(root);
        int tmp = first->val;
        first->val = second->val;
        second->val = tmp;
    }

    void inorderTraversal(TreeNode* root) {
        if (root == NULL)
            return;

        inorderTraversal(root->left);
        if (pre != NULL && pre->val > root->val) {
            if (first == NULL) {     // The swap can occur between two adjacent values or not adjacent values.
                first = pre;
                second = root;
            }
            else
                second = root;
        }
        pre = root;
        inorderTraversal(root->right);
    }
};
