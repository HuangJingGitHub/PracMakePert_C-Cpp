// DP, similar to Problem 62, from final to start
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

// Solution 2, sequential flow from start to final
class Solution {
public:
    int minPathSum(vector<vector<int>>& grid) {
        if (grid.size() == 0 || grid[0].size() == 0)
            return 0;

        int row = grid.size(), col = grid[0].size();
        for (int i = 1; i < col; i++) grid[0][i] += grid[0][i-1];
        for (int i = 1; i < row; i++) grid[i][0] += grid[i-1][0];

        for (int i = 1; i < row; i++)
            for (int j = 1; j < col; j++)
                grid[i][j] += min(grid[i-1][j], grid[i][j-1]);
        
        return grid[row-1][col-1];
    }
};
