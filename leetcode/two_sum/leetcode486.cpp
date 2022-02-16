class Solution {
public:
    bool PredictTheWinner(vector<int>& nums) {
        vector<vector<int>> dp(nums.size(), vector<int>(nums.size(), 0));
        
        // dp[i][j] = max(nums[i] - dp[i + 1][j], nums[j] - dp[i][j - 1]);
        for (int i = 0; i < nums.size(); i++)
            dp[i][i] = nums[i];
        for (int i = nums.size() - 2; i >= 0; i--) 
            for (int j = i + 1; j < nums.size(); j++)
                dp[i][j] = max(nums[i] - dp[i + 1][j], nums[j] - dp[i][j - 1]);

        return dp[0][nums.size() - 1] >= 0;
    }
};
