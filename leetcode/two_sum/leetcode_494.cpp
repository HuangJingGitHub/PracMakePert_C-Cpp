class Solution {
public:
    int findTargetSumWays(vector<int>& nums, int target) {
       return dfs(nums, target, 0, 0); 
    }

    int dfs(vector<int>& nums, int target, int idx, int currentSum) {
        if (idx == nums.size())
            return currentSum == target ? 1 : 0;
        
        int positiveRes = dfs(nums, target, idx + 1, currentSum + nums[idx]);
        int negativeRes = dfs(nums, target, idx + 1, currentSum - nums[idx]);
        return positiveRes + negativeRes;
    }
};
