// The dp state transformation equition in this problem needs some derivation. Refer to the favorited solution.
class Solution {
public:
    int numTilings(int N) {
        int modNum = 1000000007;
        vector<int> dp{1, 2, 5, 0};
        if (N <= 3)
            return dp[N - 1];

        for (int i = 4; i <= N; i++){
            dp[3] = (2 * dp[2]) % modNum + dp[0] % modNum;  // For each term just module to ensure not exceeding int range.
            for (int j = 0; j < 3; j++)
                dp[j] = dp[j + 1] % modNum; 
        }
        return dp.back() % modNum;
    }
};


class Solution {
public:
    int numTilings(int N) {
        if (N == 1)
            return 1;
        int modNum = 1000000007;
        vector<long long int> dp(N + 1), cornerDp(N + 1);
        dp[1] = 1;
        dp[2] = 2;
        cornerDp[1] = 0;
        cornerDp[2] = 2;  

        // Consider how the last tile is put. Corner represents styles with one row filled, one row not.
        for (int i = 3; i <= N; i++) {
            dp[i] = dp[i - 1] + dp[i - 2] + cornerDp[i - 1];
            cornerDp[i] = 2 * dp[i - 2] + cornerDp[i - 1];
            dp[i] %= modNum;
            cornerDp[i] %= modNum;
        }

        return dp.back() % modNum;
    }
};
