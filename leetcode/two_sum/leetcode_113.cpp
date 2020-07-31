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
    vector<vector<int>> res;
    vector<vector<int>> pathSum(TreeNode* root, int sum) {
        if (root == NULL)
            return res;
        vector<int> pathStart;
        nodeTraversal(pathStart, root, sum);
        return res;
    }

    void nodeTraversal(vector<int>& path, TreeNode* nodePt, int sum)
    {
        if (!nodePt->left && !nodePt->right){
            if (nodePt->val == sum){
                path.push_back(nodePt->val);
                res.push_back(path);
            }
        }
        else if (nodePt->left && !nodePt->right){
            path.push_back(nodePt->val);
            nodeTraversal(path, nodePt->left, sum - nodePt->val);
        }
        else if (!nodePt->left && nodePt->right){
            path.push_back(nodePt->val);
            nodeTraversal(path, nodePt->right, sum - nodePt->val);
        }
        else{
            path.push_back(nodePt->val);
            vector<int> newPath = path;
            nodeTraversal(path, nodePt->left, sum - nodePt->val);
            nodeTraversal(newPath, nodePt->right, sum - nodePt->val);
        }
    }
};
