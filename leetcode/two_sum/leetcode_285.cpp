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
  TreeNode* findSuccessor(TreeNode* root, int num) {
      TreeNode *res = nullPtr;
      inOrderFind(root, num, res);
      
      return res;
  }
  
  void inOrderFind(TreeNode* curNode, int refVal, TreeNode* &res) {
      if (curNode == nullptr)
          return;
      
      inOrderFind(curNode->left, refVal, res);
      if (curNode->val > refVal && res == nullptr) {
          res = curNode;
          return;
      }
      inOrderFind(curNode->right, refVal, res);
  }
};
