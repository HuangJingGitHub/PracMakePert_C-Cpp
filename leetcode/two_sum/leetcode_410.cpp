class Solution {
public:
    int splitArray(vector<int>& nums, int k) {
        int n = nums.size(), res = 0;
        vector<long long> preSum(n, 0);
        preSum[0] = nums[0];
        for (int i = 1; i < n; i++)
            preSum[i] = preSum[i - 1] + nums[i];

        vector<vector<long long>> dp(n, vector<long long>(k + 1, 0));
        for (int i = 0; i < n; i++) {
            for (int j = 1; j <= min(i + 1, k); j++) {
                if (j == 1) {
                    dp[i][j] = preSum[i];
                    continue;
                }
                long long tempRes = LLONG_MAX;
                for (int s = j - 1 - 1; s < i; s++)
                    tempRes = min(tempRes, max(dp[s][j - 1], preSum[i] - preSum[s]));
                dp[i][j] = tempRes;
            }
        }

        return dp.back().back();
    }
};
