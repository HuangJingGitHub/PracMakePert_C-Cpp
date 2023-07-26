// dfs
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

// dp
class Solution {
public:
    int findTargetSumWays(vector<int>& nums, int target) {
        int sum = 0;
        for (int& num : nums)
            sum += num;
        
        if (sum < abs(target))
            return 0;
        
        int len = nums.size(), range = 2 * sum + 1;
        vector<vector<int>> dp(len, vector<int>(range, 0));
        dp[0][sum + nums[0]] += 1;
        dp[0][sum - nums[0]] += 1;

        for (int i = 1; i < len; i++)
            for (int j  = -sum; j <= sum; j++) {
                if (j + nums[i] > sum)
                    dp[i][j + sum] = dp[i - 1][j - nums[i] + sum];
                else if (j - nums[i] < -sum)
                    dp[i][j + sum] = dp[i - 1][j + nums[i] + sum];
                else
                    dp[i][j + sum] = dp[i - 1][j - nums[i] + sum] + dp[i - 1][j + nums[i] + sum];
            }
        return dp.back()[target + sum];
    }
};
