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
class BSTIterator {
private:
    stack<TreeNode*> nodeStack;
    TreeNode* curNode;
public:
    BSTIterator(TreeNode* root) {
        while (root) {
            nodeStack.push(root);
            root = root->left;
        }
    }
    
    int next() {
        curNode = nodeStack.top();
        nodeStack.pop();
        int res = curNode->val;
        curNode = curNode->right;
        while (curNode) {
            nodeStack.push(curNode);
            curNode = curNode->left;
        }
        return res;
    }
    
    bool hasNext() {
        return !nodeStack.empty();
    }
};

/**
 * Your BSTIterator object will be instantiated and called as such:
 * BSTIterator* obj = new BSTIterator(root);
 * int param_1 = obj->next();
 * bool param_2 = obj->hasNext();
 */
