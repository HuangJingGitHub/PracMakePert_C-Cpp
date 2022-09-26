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
    TreeNode* first = NULL;
    TreeNode* second = NULL;
    TreeNode* pre = NULL;
    
    void recoverTree(TreeNode* root) {
        inorderTraversal(root);
        int tmp = first->val;
        first->val = second->val;
        second->val = tmp;
    }

    void inorderTraversal(TreeNode* root) {
        if (root == NULL)
            return;

        inorderTraversal(root->left);
        if (pre != NULL && pre->val > root->val) {
            if (first == NULL) {     // The swap can occur between two adjacent values or not adjacent values.
                first = pre;
                second = root;
            }
            else
                second = root;
        }
        pre = root;
        inorderTraversal(root->right);
    }
};


class Solution {
public:
    void recoverTree(TreeNode* root) {
        vector<TreeNode*>  nodeVec;

        inorder(root, nodeVec);  // 1,2,3,4,5 --> 1,4,3,2,5
        TreeNode *greaterNode = nullptr, *smallerNode = nullptr;
        for (int i = 0; i < nodeVec.size() - 1; i++) {
            if (nodeVec[i]->val > nodeVec[i + 1]->val && greaterNode == nullptr)
                greaterNode = nodeVec[i];
            if (nodeVec[i + 1]->val < nodeVec[i]->val)
                smallerNode = nodeVec[i + 1];
        }
        
        int temp = greaterNode->val;
        greaterNode->val = smallerNode->val;
        smallerNode->val = temp;
    }

    void inorder(TreeNode* root, vector<TreeNode*>& nodeVec) {
        if (root == nullptr)
            return;
        
        inorder(root->left, nodeVec);
        nodeVec.push_back(root);
        inorder(root->right, nodeVec);
    }
};
