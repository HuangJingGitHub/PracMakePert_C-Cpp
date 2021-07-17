class Solution {
public:
    int lengthOfLIS(vector<int>& nums) {
        vector<int> dp(nums.size(), 1);

        for (int i = 1; i < nums.size(); i++) {
            int preLongest = 0;
            for (int j = i - 1; j >= 0; j--) {
                if (nums[j] < nums[i])
                    preLongest = max(preLongest, dp[j]);
            }
                dp[i] = preLongest + 1;
        }
        int res = 0;
        for (int& num : dp)
            res = max(res, num);
        return res;
    }
};
