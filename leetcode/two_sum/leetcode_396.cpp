class Solution {
public:
    int maxRotateFunction(vector<int>& nums) {
        int res = INT_MIN, sum = 0, n = nums.size(), F = 0;

        for (int& num : nums)
            sum += num;
        for (int i = 0; i < n; i++) {
            F += i * nums[i];
        }
        res = max(res, F);
        for (int i = 1; i < n; i++) {
            F = F + sum - n * nums[n - i];
            res = max(res, F);
        }
        return res;
    }
};
