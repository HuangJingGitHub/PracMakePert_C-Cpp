class Solution {
public:
    int combinationSum4(vector<int>& nums, int target) {
        vector<int> dp(target + 1, 0);
        dp[0] = 1;
        
        for (int i = 1; i <= target; i++) {
            for (int num : nums) {
                if (i - num >= 0 && INT_MAX - dp[i - num] > dp[i])  // just to avoid exceed int bound
                    dp[i] += dp[i - num];
            }
        }
        
        return dp.back();
    }
};