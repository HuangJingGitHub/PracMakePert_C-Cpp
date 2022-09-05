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
    int sumNumbers(TreeNode* root) {
        int res  = 0;
        queue<int> levelNum;
        queue<TreeNode*> levelNode;

        levelNum.push(root->val);
        levelNode.push(root);
        while (levelNode.empty() == false) {
            int levelLen = levelNode.size();
            while (levelLen > 0) {
                int curNum = levelNum.front();
                levelNum.pop();
                TreeNode* curNode = levelNode.front();
                levelNode.pop();

                if (curNode->left) {
                    levelNode.push(curNode->left);
                    levelNum.push(curNode->left->val + curNum * 10);
                }
                if (curNode->right) {
                    levelNode.push(curNode->right);
                    levelNum.push(curNode->right->val + curNum * 10);
                }
                if (curNode->left == nullptr && curNode->right == nullptr)
                    res += curNum;
                levelLen--;
            }
        }
        
        return res;
    }
};
