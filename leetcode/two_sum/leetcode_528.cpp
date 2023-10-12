class Solution {
public:
    bool checkSubarraySum(vector<int>& nums, int k) {
        vector<int> sum(nums.size(), 0);
        sum[0] = nums[0];

        for (int i = 1; i < nums.size(); i++)
            sum[i] = nums[i] + sum[i - 1];

        for (int i = 1; i < sum.size(); i++) {
            if (sum[i] % k == 0)
                return true;
            for (int j = 0; j < i - 1; j++) {
                if ((sum[i] - sum[j]) % k == 0)
                    return true;
            }
        }
        return false;
    }
};
