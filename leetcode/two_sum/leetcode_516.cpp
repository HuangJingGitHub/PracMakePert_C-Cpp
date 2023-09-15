class Solution {
public:
    int longestPalindromeSubseq(string s) {
        int len = s.size(), res = 1;
        vector<vector<int>> dp(len, vector<int>(len, 0));
    
        for (int i = len - 1; i >= 0; i--)
            for (int j = i; j < len; j++) {
                if (i == j) {
                    dp[i][j] = 1;
                    continue;
                }
                else {
                    if (s[i] == s[j])
                        dp[i][j] = dp[i + 1][j - 1] + 2;
                    else 
                        dp[i][j] = max(dp[i][j - 1], dp[i + 1][j]);
                }
                res = max(res, dp[i][j]);
            }

        return res;
    }
};
