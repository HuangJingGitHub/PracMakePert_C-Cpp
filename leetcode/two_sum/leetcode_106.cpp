// Similar to Problem 105
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
    TreeNode* buildTree(vector<int>& inorder, vector<int>& postorder) {
        return buildTreeHelper(postorder, 0, postorder.size(), inorder, 0, inorder.size());
    }

    TreeNode* buildTreeHelper(vector<int>& postorder, int pStart, int pEnd, vector<int>& inorder, int iStart, int iEnd) {
        if (pStart == pEnd)
            return NULL;
        
        TreeNode* root = new TreeNode(postorder[pEnd - 1]);

        int inorderRootIdx = 0;
        for (int i = iStart; i < iEnd; i++) {
            if (inorder[i] == postorder[pEnd-1]) {
                inorderRootIdx = i;
                break;
            }
        }

        int leftChildNum = inorderRootIdx - iStart;
        root->left = buildTreeHelper(postorder, pStart, pStart+leftChildNum, inorder, iStart, iStart+leftChildNum);
        root->right = buildTreeHelper(postorder, pStart+leftChildNum, pEnd-1, inorder, inorderRootIdx+1, iEnd);
        return root;
    }
};
