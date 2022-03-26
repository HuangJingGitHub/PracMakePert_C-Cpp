class Solution {
public:
    int wiggleMaxLength(vector<int>& nums) {
        if (nums.size() <= 1)
            return nums.size();

        vector<int> dp(nums.size(), 0), signLog(nums.size(), 0);
        dp[0] = 1;
        int i = 1;
        while (i < nums.size() && nums[i] == nums[i - 1])
            i++;
        if (i == nums.size())
            return 1;
        signLog[i] = nums[i] - nums[0] > 0 ? 1 : -1;
        dp[i] = 2;

        for (i++; i < nums.size(); i++) {
            if (nums[i] == nums[i - 1]) {
                dp[i] = dp[i - 1];
                signLog[i] = signLog[i - 1];
                continue;
            }
            int curSign = nums[i] - nums[i - 1] > 0 ? 1 : -1;
            if (curSign * signLog[i - 1] == 1) {
                dp[i] = dp[i - 1];
                signLog[i] = signLog[i - 1];
            }
            else {
                dp[i] = dp[i - 1] + 1;
                signLog[i] = curSign;
            }
        }
        return dp.back();
    }
};
