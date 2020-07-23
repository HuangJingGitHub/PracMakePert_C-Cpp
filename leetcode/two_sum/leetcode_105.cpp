// Refer to the solution sections. recursion
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
    TreeNode* buildTree(vector<int>& preorder, vector<int>& inorder) {
        return buildTreeHelper(preorder, 0, preorder.size(), inorder, 0, inorder.size());
    }

    TreeNode* buildTreeHelper(vector<int>& preorder, int pStart, int pEnd, vector<int>& inorder, int iStart, int iEnd)
    {
        if (pStart == pEnd)
            return NULL;
        
        int rootVal = preorder[pStart];
        TreeNode* root = new TreeNode(rootVal);

        int inorderRootIdx = 0;
        for (int i = iStart; i < iEnd; i++){
            if (rootVal == inorder[i]){
                inorderRootIdx = i;
                break;
            }
        }

        int leftNum = inorderRootIdx - iStart;
        root->left = buildTreeHelper(preorder, pStart+1, pStart+leftNum+1, inorder, iStart, inorderRootIdx);
        root->right = buildTreeHelper(preorder, pStart+leftNum+1, pEnd, inorder, inorderRootIdx+1, iEnd);
        return root;
    }
};
