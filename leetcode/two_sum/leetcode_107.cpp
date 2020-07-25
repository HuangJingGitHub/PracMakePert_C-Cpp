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
    vector<vector<int>> levelOrderBottom(TreeNode* root) {
        vector<vector<int>> res;
        if (root == NULL)
            return res;

        vector<int> levelVal;
        queue<TreeNode*> levelNode;
        levelNode.push(root);
        int levelNodeNum;

        while (!levelNode.empty()){
            levelNodeNum = levelNode.size();
            for (int i = 0; i < levelNodeNum; i++){
                TreeNode* x = levelNode.front();
                levelVal.push_back(x->val);
                if (x->left != NULL)
                    levelNode.push(x->left);
                if (x->right != NULL)
                    levelNode.push(x->right);
                levelNode.pop();
            }
            // res.insert(res.begin(), levelVal);
            res.push_back(levelVal);
            levelVal.clear();
        }
        reverse(res.begin(), res.end());   // Much much more efficient than inserting at the beginning, because push_back() is much more fast than insert() for vector.
        return res;
    }
};
