class Solution {
public:
    vector<int> largestDivisibleSubset(vector<int>& nums) {
        vector<int> res, dp(nums.size(), 1);
        sort(nums.begin(), nums.end());

        int resLen = 1, resMax = nums[0];
        for (int i = 1; i < nums.size(); i++) {
            for (int j = i - 1; j >= 0; j--)
                if (nums[i] % nums[j] == 0) {
                    dp[i] = max(dp[i], dp[j] + 1);
                    if (dp[i] > resLen) {
                        resLen = dp[i];
                        resMax = nums[i];
                    }
                }
        }
        
        res = vector<int>(resLen, 0);
        for (int i = nums.size() - 1; i >= 0; i--) {
            if (dp[i] == resLen && resMax % nums[i] == 0) {
                res[resLen - 1] = nums[i];
                resLen--;
                resMax = nums[i];
            }
        }

        return res;
    }
};
