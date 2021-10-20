class Solution {
public:
    int maxProduct(vector<int>& nums) {
        int imax = nums[0], imin = nums[0], res = nums[0];
        for (int i = 1; i < nums.size(); i++) {
            if (nums[i] < 0)   // This process is cool.
                swap(imax, imin);
            imax = max(imax * nums[i], nums[i]);
            imin = min(imin * nums[i], nums[i]);
            res = max(imax, res);
        }
        return res;
    }
};


class Solution {
public:
    int maxProduct(vector<int>& nums) {
        int res = nums[0];
        vector<vector<int>> dp(nums.size(), vector<int>(2));
        dp[0][0] = nums[0];
        dp[0][1] = nums[0];

        for (int i = 1; i < nums.size(); i++) {
            if (nums[i] >= 0) {
                dp[i][0] = min(nums[i], nums[i] * dp[i - 1][0]);
                dp[i][1] = max(nums[i], nums[i] * dp[i - 1][1]);
            }
            else {
                dp[i][0] = min(nums[i], nums[i] * dp[i - 1][1]);
                dp[i][1] = max(nums[i], nums[i] * dp[i - 1][0]); 
            }
        }

        for (int i = 1; i < nums.size(); i++)
            res = max(res, dp[i][1]);
        return res;
    }
};
