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
    vector<vector<int>> zigzagLevelOrder(TreeNode* root) {
        vector<vector<int>> res;
        if (root == nullptr)
            return res;

        deque<TreeNode*> levelNode;
        int levelSize;
        levelNode.push_back(root);
        bool leftToRight = true;

        while (levelNode.empty() == false) {
            levelSize = levelNode.size();
            vector<int> levelRes;
            TreeNode* curNode;

            if (leftToRight == true) {
                for (int i = 0; i < levelSize; i++) {
                    curNode = levelNode.back();
                    levelNode.pop_back();
                    levelRes.push_back(curNode->val);
                    if (curNode->left)
                        levelNode.push_front(curNode->left);
                    if (curNode->right)
                        levelNode.push_front(curNode->right);
                }
            }
            else {
                for (int i = 0; i < levelSize; i++) {
                    curNode = levelNode.front();
                    levelNode.pop_front();
                    levelRes.push_back(curNode->val);
                    if (curNode->right)
                        levelNode.push_back(curNode->right);
                    if (curNode->left)
                        levelNode.push_back(curNode->left);                    
                }
            }
            res.push_back(levelRes);
            leftToRight = !leftToRight;
        }
        return res;
    }
};

// simpler
class Solution {
public:
    vector<vector<int>> zigzagLevelOrder(TreeNode* root) {
        vector<vector<int>> res;
        if (root == nullptr)
            return res;

        queue<TreeNode*> levelNode;
        int levelSize;
        levelNode.push(root);
        bool leftToRight = true;

        while (levelNode.empty() == false) {
            levelSize = levelNode.size();
            vector<int> levelRes(levelSize, 0);
            TreeNode* curNode;

            for (int i = 0; i < levelSize; i++) {
                curNode = levelNode.front();
                levelNode.pop();
                int idx = leftToRight ? i : levelSize - 1 - i;
                levelRes[idx] = curNode->val;
                if (curNode->left)
                    levelNode.push(curNode->left);
                if (curNode->right)
                    levelNode.push(curNode->right);
            }
            res.push_back(levelRes);
            leftToRight = !leftToRight;
        }
        return res;
    }
};
