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
    long int target;
    unordered_map<long int, int> um;

    int pathSum(TreeNode* root, int targetSum) {
        target = targetSum;
        um[0] = 1;
        return dfs(root, 0);
    }


    int dfs(TreeNode* node, long int curSum) {
        if (node == nullptr)
            return 0;

        int res = 0;
        curSum += node->val;
        res += um[curSum - target];
        um[curSum] += 1;
        
        res += dfs(node->left, curSum);
        res += dfs(node->right, curSum);
        um[curSum] -= 1;
        return res;
    }
};
