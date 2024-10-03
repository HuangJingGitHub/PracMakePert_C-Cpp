class Solution {
public:
    vector<vector<int>> directions{{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    int MOD = 1000000007;

    int findPaths(int m, int n, int maxMove, int startRow, int startColumn) {
        vector<vector<vector<int>>> dp(m, vector<vector<int>>(n, vector<int>(maxMove + 1, 0)));
        for (int move = 1; move <= maxMove; move++) 
            for (int row = 0; row < m; row++)
                for (int col = 0; col < n; col++) {
                    // How the boundaries are processed avoids 
                    // tedious processing by cases.
                    if (row == 0)
                        dp[row][col][move]++;
                    if (col == 0)
                        dp[row][col][move]++;
                    if (row == m - 1)
                        dp[row][col][move]++;
                    if (col == n - 1)
                        dp[row][col][move]++;
                    
                    for (vector<int>& dir : directions) {
                        int next_row = row + dir[0],
                            next_col = col + dir[1];
                        if (next_row >= 0 && next_row < m && next_col >= 0 && next_col < n)
                            dp[row][col][move] = (dp[row][col][move] + dp[next_row][next_col][move - 1]) % MOD;
                    }
                }
        return dp[startRow][startColumn][maxMove];
    }
};
