// DP, similar to Problem 62.
class Solution {
public:
    int minPathSum(vector<vector<int>>& grid) {
        if (grid.size() == 0 || grid[0].size() == 0)
            return 0;

        vector<int> dp = grid.back();
        int row = grid.size(), col = grid[0].size();
        for (int i = col - 2; i >= 0; i--) dp[i] += dp[i+1];

        for (int i = row - 2; i >= 0; i--){
            dp[col-1] += grid[i][col-1];
            for (int j = col - 2; j >= 0; j--)
                    dp[j] = (min(dp[j], dp[j+1]) + grid[i][j]);
        }

        return dp[0];
    }
};
