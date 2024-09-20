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
    bool res = false;
    bool isSubtree(TreeNode* root, TreeNode* subRoot) { 
        inOrderCheck(root, subRoot);
        return res;
    }
    
    void inOrderCheck(TreeNode* root, TreeNode* subRoot) {
        if (root == nullptr)
            return;

        inOrderCheck(root->left, subRoot);
        if (root->val == subRoot->val)
            if (isEqualTree(root, subRoot) == true)
                res = true;
        if (res == true)
            return;
        inOrderCheck(root->right, subRoot);
    }

    bool isEqualTree(TreeNode* root_1, TreeNode* root_2) {
        if (root_1 == nullptr && root_2 == nullptr)
            return true;
        if (root_1 == nullptr && root_2 != nullptr)
            return false;
        if (root_1 != nullptr && root_2 == nullptr)
            return false;
        if (root_1->val != root_2->val)
            return false;
        return 
            isEqualTree(root_1->left, root_2->left) && isEqualTree(root_1->right, root_2->right);
    }
};


// recursion 
class Solution {
public:
    bool isSubtree(TreeNode* root, TreeNode* subRoot) { 
        if (isEqualTree(root, subRoot))
            return true;
        if (root == nullptr)
            return false;
        return isSubtree(root->left, subRoot) || isSubtree(root->right, subRoot);
    }

    bool isEqualTree(TreeNode* root_1, TreeNode* root_2) {
        if (root_1 == nullptr && root_2 == nullptr)
            return true;
        return root_1 && root_2 &&
               root_1->val == root_2->val &&
               isEqualTree(root_1->left, root_2->left) &&
               isEqualTree(root_1->right, root_2->right);
    }
};
