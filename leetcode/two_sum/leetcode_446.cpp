class Solution {
public:
    int numberOfArithmeticSlices(vector<int>& nums) {
        int res = 0, len = nums.size();

        vector<unordered_map<long long, int>> dp(len);
        for (int i = 1; i < len; i++)
            for (int j = i - 1; j >= 0; j--) {
                long long delta = (long long) nums[i] - nums[j];
                dp[i][delta]++;
                if (dp[j].find(delta) != dp[j].end()) {
                    dp[i][delta] += dp[j][delta];
                    res += dp[j][delta];
                }
            }
        
        return res;
    }
};
