// Similar to Problem 102, but just pay attention to the order. Cannot use queue but vector or stack.
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
    vector<vector<int>> zigzagLevelOrder(TreeNode* root) {
        vector<vector<int>> res;
        if (root == NULL)
            return res;
        bool direction = true; // left ---> right is true. 
        vector<TreeNode*> levelNode;
        levelNode.push_back(root);
    
        vector<int> levelVal;
        vector<TreeNode*> nextLevel;
        TreeNode* tmp;
        
        while (!levelNode.empty()){    
            for (int i = levelNode.size()-1; i >= 0; i--){
                tmp = levelNode[i];
                levelVal.push_back(tmp->val);
                if (direction){
                    if (tmp->left != NULL)
                        nextLevel.push_back(tmp->left);
                    if (tmp->right != NULL)
                        nextLevel.push_back(tmp->right);
                }
                else{
                    if (tmp->right != NULL)
                        nextLevel.push_back(tmp->right);
                    if (tmp->left != NULL)
                        nextLevel.push_back(tmp->left);                        
                }
            }

            res.push_back(levelVal);
            levelNode = nextLevel;
            direction = !direction;
            levelVal.clear();
            nextLevel.clear();
        }
        return res;
    }
};
