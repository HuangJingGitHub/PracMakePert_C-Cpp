class Solution {
public:
    int change(int amount, vector<int>& coins) {
        vector<int> dp(amount + 1, 0);
        dp[0] = 1;
        for (int coin : coins)
            for (int i = 0; i + coin <= amount; i++) {
                dp[i + coin] += dp[i];
            }
        return dp.back();
    }
};
