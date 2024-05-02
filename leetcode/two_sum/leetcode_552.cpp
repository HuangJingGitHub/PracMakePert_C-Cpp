class Solution {
public:
    int mod_num = 1e9 + 7;

    int checkRecord(int n) {
        long long dp[n][2][3];
        std::fill_n(&dp[0][0][0], n * 2 * 3, (long)0);
        dp[0][0][0] = 1;
        dp[0][1][0] = 1;
        dp[0][0][1] = 1;

        for (int i = 1; i < n; i++) {
            dp[i][0][0] = (dp[i - 1][0][0] + dp[i - 1][0][1] + dp[i - 1][0][2]) % mod_num;
            dp[i][0][1] = dp[i - 1][0][0] % mod_num;
            dp[i][0][2] = dp[i - 1][0][1] % mod_num;
            dp[i][1][0] = (dp[i - 1][0][0] + dp[i - 1][0][1] + dp[i - 1][0][2] + dp[i - 1][1][0] 
                            + dp[i - 1][1][1] + dp[i - 1][1][2]) % mod_num;
            dp[i][1][1] = dp[i - 1][1][0] % mod_num;
            dp[i][1][2] = dp[i - 1][1][1] % mod_num;
        }

        long res = 0;
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 3; j++)
                res += dp[n - 1][i][j];
        return res % mod_num;
    }
};
