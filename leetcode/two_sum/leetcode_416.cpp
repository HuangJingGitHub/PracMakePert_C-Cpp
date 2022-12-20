class Solution {
public:
    bool canPartition(vector<int>& nums) {
        int sum = 0;
        for (int& num : nums)
            sum += num;
        if (sum % 2 != 0)
            return false;
        
        int target = sum / 2;
        vector<vector<bool>> dp(nums.size() + 1, vector<bool>(target + 1, false));
        for (int i = 0; i <= nums.size(); i++)
            dp[i][0] = true;
        
        for (int i = 1; i <= nums.size(); i++)
            for (int j = 1; j <= target; j++) {
                if (dp[i - 1][j] == true || (j - nums[i - 1] >= 0 && dp[i - 1][j - nums[i - 1]] == true))
                    dp[i][j] = true;
            }
        
        return dp.back().back();
    }
};
