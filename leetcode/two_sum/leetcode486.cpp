class Solution {
public:
    bool PredictTheWinner(vector<int>& nums) {
        vector<vector<int>> dp(nums.size(), vector<int>(nums.size(), 0));
        
        // dp[i][j] = max(nums[i] - dp[i + 1][j], nums[j] - dp[i][j - 1]);
        for (int i = 0; i < nums.size(); i++)
            dp[i][i] = nums[i];
        for (int row = nums.size() - 2; row >= 0; row--) 
            for (int col = row + 1; col < nums.size(); col++)
                dp[row][col] = max(nums[row] - dp[row + 1][col], nums[col] - dp[row][col - 1]);

        return dp[0][nums.size() - 1] >= 0;
    }
};
