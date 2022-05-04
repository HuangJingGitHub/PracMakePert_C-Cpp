class Solution {
public:
    int uniquePathsWithObstacles(vector<vector<int>>& obstacleGrid) {
        int m = obstacleGrid.size(), n = obstacleGrid[0].size();
        vector<vector<int>> dp(m, vector(n, 0));

        if (obstacleGrid[0][0] == 1)
            return 0;
        dp[0][0] = 1;
        for (int col = 1; col < n; col++)
            if (obstacleGrid[0][col] == 1)
                dp[0][col] = 0;
            else
                dp[0][col] = dp[0][col-1];
        for (int row = 1; row < m; row++)
            if (obstacleGrid[row][0] == 1)
                dp[row][0] = 0;
            else
                dp[row][0] = dp[row-1][0];

        for (int row = 1; row < m; row++)
            for (int col = 1; col < n; col++)
                if (obstacleGrid[row][col] == 1)
                    dp[row][col] = 0;
                else
                    dp[row][col] = dp[row-1][col] + dp[row][col-1]; 

        return dp[m-1][n-1];
    }
};


class Solution {
public:
    int uniquePathsWithObstacles(vector<vector<int>>& obstacleGrid) {
        if (obstacleGrid[0][0] == 1 || obstacleGrid.back().back() == 1)
            return 0;

        int rowNum = obstacleGrid.size(), colNum = obstacleGrid[0].size();
        obstacleGrid[0][0] = 1;
        for (int col = 1; col < colNum; col++)
            obstacleGrid[0][col] = (obstacleGrid[0][col] == 1 ? 0 : obstacleGrid[0][col - 1]);
        for (int row = 1; row < rowNum; row++)
            obstacleGrid[row][0] = (obstacleGrid[row][0] == 1 ? 0 : obstacleGrid[row - 1][0]);
        
        for (int row = 1; row < rowNum; row++)
            for (int col = 1; col < colNum; col++) {
                if (obstacleGrid[row][col] == 1)
                    obstacleGrid[row][col] = 0;
                else
                    obstacleGrid[row][col] = obstacleGrid[row - 1][col] + obstacleGrid[row][col - 1];
            }

        return obstacleGrid.back().back();
    }
};
