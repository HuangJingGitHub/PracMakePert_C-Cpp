class Solution {
public:
    int res;

    int kthSmallest(TreeNode* root, int k) {
        int cnt = 1;
        inorder(root, k, cnt);
        return res;
    }
    
    void inorder(TreeNode* node, int k, int& cnt) {
        if (node == NULL)
            return;
        inorder(node->left, k, cnt);
        if (cnt == k) {
            res = node->val;
            cnt++;  // Self-increment on cnt here to avoid res to be reassigned in recursive funciton return.
            return;
        }
        cnt++;
        inorder(node->right, k, cnt);
    }
};
