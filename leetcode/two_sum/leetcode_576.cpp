class Solution {
public:
    int findPaths(int m, int n, int maxMove, int startRow, int startColumn) {
        vector<vector<int>> dp(m, vector<int>(n, 0)), preDp;
        for (int col = 1; col < n - 1; col++) {
            dp[0][col] = 1;
            dp.back()[col] = 1;
        }      
        for (int row = 1; row < m - 1; row++) {
            dp[row][0] = 1;
            dp[row].back() = 1;
        }
        dp[0][0] = 2;
        dp[0].back() = 2;
        dp.back()[0] = 2;
        dp.back().back() = 2;

        for (int move = 2; move <= maxMove; move++){
            preDp = dp;
            for (int col = 1; col < n - 1; col++) {
                dp[0][col] = preDp[0][col - 1] + preDp[0][col + 1];
                if (m > 1)
                    dp[0][col] += preDp[1][col];
                dp.back()[col] = preDp.back()[col - 1] + preDp.back()[col + 1];
                if (m > 1)
                    dp.back()[col] += preDp[m - 2][col];
            }
            for (int row = 1; row < m - 1; row++) {
                dp[row][0] = preDp[row - 1][0] + preDp[row + 1][0];
                if (n > 1)
                    dp[row][0] += preDp[row][1];
                dp[row].back() = preDp[row - 1].back() + preDp[row + 1].back();
                if (n > 1)
                    dp[row].back() += preDp[row][n - 2];
            }

            dp[0][0] = 0;
            dp[0].back() = 0;
            dp.back()[0] = 0;
            dp.back().back() = 0;
            if (n > 1) {
                dp[0][0] += preDp[0][1];
                dp[0].back() += preDp[0][n - 2];
                if (m > 1) {
                    dp.back()[0] += preDp.back()[1];
                    dp.back().back() =+ preDp.back()[n - 2];
                }
            }
            if (m > 1){
                dp[0][0] += preDp[1][0];
                dp[0].back() += preDp[1].back();
                if (n > 1) {
                    dp.back()[0] += preDp[m - 2][0];
                    dp.back().back() =+ preDp[m - 2].back();
                }               
            }

            for (int row = 1; row < m - 1; row++)
                for (int col = 1; col < n - 1; col++) {
                    dp[row][col] = preDp[row - 1][col] + preDp[row + 1][col]
                                + preDp[row][col - 1] + preDp[row][col + 1];
                }
        }
        return dp[startRow][startColumn];
    }
};
