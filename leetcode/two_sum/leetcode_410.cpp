class Solution {
public:
    int splitArray(vector<int>& nums, int k) {
        int n = nums.size(), res = 0;
        vector<int> preSum(n, 0);
        preSum[0] = nums[0];
        for (int i = 1; i < n; i++)
            preSum[i] = preSum[i - 1] + nums[i];

        vector<vector<int>> dp(n, vector<int>(k + 1, 0));
        dp[0][1] = nums[0];
        for (int i = 1; i < n; i++)
            for (int j = 1; j <= min(i, k); j++) {
                int tempRes = INT_MAX;
                for (int s = j - 1; s <= i - 1; s++) {
                    tempRes = min(tempRes, max(dp[s][j - 1], preSum[j] - preSum[s]));
                }
                dp[i][j] = tempRes;
            }
        for (auto vec : dp) {
            cout << '\n';
            for (auto x : vec)
                cout << x << " ";
        } 

        return dp.back().back();
    }
};
