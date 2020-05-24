// Using permutation and combination the result is C_{m+n-2}^{m-1}
class Solution {
public:
    // Recursion can be very slow
    int uniquePathsSelf(int m, int n) {
        if (m == 1 || n == 1)
            return 1;
        else
            return uniquePathsSelf(m, n-1) + uniquePathsSelf(m-1, n);    
    }
    // Interesting DP
    int uniquePaths(int m, int n) {
    vector<int> dp(m,0);
    dp[0] = 1;
    for(int i  = 0; i < n; i++)
        for(int j = 1; j < m; j++)
            dp[j] += dp[j-1];
    return dp[m-1];
    }
};
