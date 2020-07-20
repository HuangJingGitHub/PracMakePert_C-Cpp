// Very cool but also regular implementation of BST level order traversal by using queue
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
    vector<vector<int>> levelOrder(TreeNode* root) {
        vector<vector<int>> res;
        if (root == NULL)
            return res;
        queue<TreeNode*> nodeQueue;
        nodeQueue.push(root);

        while (!nodeQueue.empty()){
            int count = nodeQueue.size();
            vector<int> level;
            while (count > 0){
                TreeNode* node = nodeQueue.front();
                nodeQueue.pop();
                level.push_back(node->val);
                if (node->left != NULL)
                    nodeQueue.push(node->left);
                if (node->right != NULL)
                    nodeQueue.push(node->right);
                count--;
            }
            res.push_back(level);
        }
        return res;
    }
};
