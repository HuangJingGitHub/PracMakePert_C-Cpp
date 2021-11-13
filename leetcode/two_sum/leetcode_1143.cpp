class Solution {
public:
    int longestCommonSubsequence(string text1, string text2) {
        int m = text1.size(), n = text2.size();
        vector<vector<int>> dp(m, vector<int>(n, 0));
        
        if (text1[0] == text2[0])
            dp[0][0] = 1;
        for (int col = 1; col < n; col++) {
            if (dp[0][col - 1] == 1)
                dp[0][col] = 1;
            else if (text2[col] == text1[0])
                dp[0][col] = 1;   
        }
        for (int row = 1; row < m; row++) {
            if (dp[row - 1][0] == 1)
                dp[row][0] = 1;
            else if (text1[row] == text2[0])
                dp[row][0] = 1;
        }

        for (int row = 1; row < m; row++)
            for (int col = 1; col < n; col++) {
                if (text1[row] == text2[col])
                    dp[row][col] = dp[row - 1][col - 1] + 1;
                else                
                    dp[row][col] = max(dp[row][col - 1], dp[row - 1][col]);
            }
        return dp.back().back();
    }
};
