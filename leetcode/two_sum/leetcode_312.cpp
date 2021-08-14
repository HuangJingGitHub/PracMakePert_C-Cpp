class Solution {
public:
    int maxCoins(vector<int>& nums) {
        int n = nums.size();
        vector<vector<int>> dp(n + 2, vector<int>(n + 2, 0));
        vector<int> temp(n + 2, 0);
        temp.front() = 1;
        temp.back() = 1;
        
        for (int i = 1; i <= n; i++)
            temp[i] = nums[i - 1];
        
        for (int len = 3; len <= n + 2; len++)
            for (int left = 0; left <= n + 2 - len; left++)
                for (int k = left + 1; k < left + len - 1; k++) {
                    int leftRes = dp[left][k],
                        rightRes = dp[k][left + len - 1];
                    int sum = leftRes + rightRes + temp[left] * temp[k] * temp[left + len - 1];
                    dp[left][left + len - 1] = max(dp[left][left + len - 1], sum); 
                }
        return dp[0][n + 1];
    }
};
