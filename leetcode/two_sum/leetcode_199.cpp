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
    vector<int> rightSideView(TreeNode* root) {
        vector<int> res;
        if (!root)
            return res;
        
        queue<TreeNode*> levelNode;
        levelNode.push(root);
        
        while (levelNode.size() != 0){
            res.push_back(levelNode.back()->val);
            
            int levelNodeNum = levelNode.size();
            for (int i = 0; i < levelNodeNum; i++){
                if (levelNode.front()->left)
                    levelNode.push(levelNode.front()->left);
                if (levelNode.front()->right)
                    levelNode.push(levelNode.front()->right);
                levelNode.pop();
            }
        }
        return res;
    }
};
