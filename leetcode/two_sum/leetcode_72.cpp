// DP is the right thought to solve the problem, quite subtle solution.
// Can refer to the discussion and solution sections on LeetCode.
class Solution {
public:
    int minDistance(string word1, string word2) {
        int m = word1.size(), n = word2.size();
        vector<vector<int>> dp(m + 1, vector<int>(n + 1, 0));

        for (int col = 0; col <= n; col++)
            dp[0][col] = col;
        for (int row = 0; row <= m; row++)
            dp[row][0] = row;
        
        for (int row = 1; row <= m; row++)
            for (int col = 1; col <= n; col++) {
                // Note row - 1, col - 1 are indices in word1 and word2, -1 is needed.
                if (word1[row - 1] == word2[col - 1])
                    dp[row][col] = dp[row - 1][col - 1];
                else
                    dp[row][col] = min(dp[row - 1][col - 1], min(dp[row - 1][col], dp[row][col - 1])) + 1;
            }
        return dp.back().back();
    }
};
