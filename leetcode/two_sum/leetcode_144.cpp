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
    vector<int> res;
    vector<int> preorderTraversal(TreeNode* root) {
        if (root) {
            res.push_back(root->val);
            preorderTraversal(root->left);
            preorderTraversal(root->right);
        }
        return res;
    }
};

 // It is cool to use a stack, actually similar to recursion (same O(n) time and space complexity), but more explicit.
class Solution {
public:
    vector<int> preorderTraversal(TreeNode* root) {
        if (!root)
            return {};
        vector<int> res;
        stack<TreeNode*> nodeStack;
        nodeStack.push(root);

        while (!nodeStack.empty()) {
            TreeNode* curNode = nodeStack.top();
            res.push_back(curNode->val);
            nodeStack.pop();
            if (curNode->right)
                nodeStack.push(curNode->right);
            if (curNode->left)
                nodeStack.push(curNode->left);  // Left node is always on the top of stack.
        }
        return res;
    }
};
