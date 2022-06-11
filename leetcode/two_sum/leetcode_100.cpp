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
    bool res = true;
    bool isSameTree(TreeNode* p, TreeNode* q) {
        if (p == NULL && q != NULL)
            return false;
        else if (p != NULL && q == NULL)
            return false;
    
        if (p && q){
            isSameTree(p->left, q->left);
            if (p->val != q->val 
                || (p->left == NULL && q->left != NULL) || (p->left != NULL && q->left == NULL) 
                || (p->right == NULL && q->right != NULL) || (p->right != NULL && q->right == NULL))
                res = false;
            else
                isSameTree(p->right, q->right);
        }
        return res;
    }
};


// More compact, elegant, and efficient code with inorder traversal and recursion.
class Solution {
public:
    bool isSameTree(TreeNode* p, TreeNode* q) {
        if (p == nullptr && q == nullptr)
            return true;
        if ((p && !q) || (!p && q))
            return false;
        if (p->val != q->val)
            return false;
        return isSameTree(p->left, q->left) && isSameTree(p->right, q->right);
    }
};
