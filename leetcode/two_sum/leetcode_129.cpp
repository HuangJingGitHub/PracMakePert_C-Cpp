// BFS
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
    int sumNumbers(TreeNode* root) {
        int res = 0;
        if (root == NULL)
            return res;

        queue<TreeNode*> levelNodes;
        levelNodes.push(root);

        while (!levelNodes.empty()){
            int levelNodesNum = levelNodes.size();
            for (int i = 0; i < levelNodesNum; i++){
                TreeNode* curNode = levelNodes.front();
                levelNodes.pop();
                if (curNode->left != NULL){
                    curNode->left->val += 10 * curNode->val;
                    levelNodes.push(curNode->left);
                }
                if (curNode->right != NULL){
                    curNode->right->val += 10 * curNode->val;
                    levelNodes.push(curNode->right);
                }
                if (curNode->left == NULL && curNode->right == NULL)
                    res += curNode->val;
            }
        }

        return res;
    }
};


// recursion
class Solution {
public:
    int sumNumbers(TreeNode* root) {
        return helper(root, 0);
    }

    int helper(TreeNode* root, int parentVal) {
        if (root == NULL)
            return 0;
        parentVal = 10 * parentVal + root->val;
        if (root->left == NULL && root->right == NULL)
            return parentVal;
        else
            return helper(root->left, parentVal) + helper(root->right, parentVal);
    }
};
