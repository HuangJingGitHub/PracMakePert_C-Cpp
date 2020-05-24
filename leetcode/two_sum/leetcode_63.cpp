class Solution {
public:
    int uniquePathsWithObstacles(vector<vector<int>>& obstacleGrid) {
        if (obstacleGrid.size() == 0 || obstacleGrid[0].size() == 0)
            return 0;

        int m = obstacleGrid[0].size(), n = obstacleGrid.size();
        vector<vector<int>> dp(n, vector(m, 0));
        vector<vector<bool>> isObstacle(n, vector(m, false));

        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                if (obstacleGrid[i][j] == 1){
                    isObstacle[i][j] = true;
                }

        for (int i = 0; i < m; i++){
            if (!isObstacle[0][i])
                dp[0][i] = 1;
            else
                break;
        } 
        
        for (int i = 0; i < n; i++){
            if (!isObstacle[i][0])
                dp[i][0] = 1;
            else
                break;
        }

        for (int i = 1; i < n; i++)
            for (int j = 1; j < m; j++){
                dp[i][j] = dp[i][j-1] + dp[i-1][j];
                if (isObstacle[i][j])
                    dp[i][j] = 0;
            }
        
        return dp[n-1][m-1];
    }
};
