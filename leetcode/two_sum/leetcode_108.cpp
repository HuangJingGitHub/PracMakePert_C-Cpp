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
    TreeNode* sortedArrayToBST(vector<int>& nums) {
        if (nums.empty())   
            return NULL;
        return helper(nums, 0, nums.size() - 1);
    }

    TreeNode* helper(vector<int>& nums, int start, int end) {
        if (start > end)
            return NULL;

        int mid = start + (end - start) / 2;    // detail: mid = (start + end) / 2 can cause overflow if start + end > INT_MAX. start + (end - start) / 2 is safer.
        TreeNode* node = new TreeNode(nums[mid]);
        node->left = buildBranch(nums, start, mid-1);
        node->right = buildBranch(nums, mid+1, end);
        return node;
    }
};
