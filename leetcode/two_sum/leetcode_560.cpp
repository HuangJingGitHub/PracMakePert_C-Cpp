class Solution {
public:
    int subarraySum(vector<int>& nums, int k) {
        int res = 0, pre_sum = 0;
        map<int, int> pre_sum_counter;
        pre_sum_counter[0] = 1;
        for (size_t i = 0; i < nums.size(); i++) {
            pre_sum += nums[i];
            if (pre_sum_counter.count(pre_sum - k) != 0)
                res += pre_sum_counter[pre_sum - k];
            pre_sum_counter[pre_sum]++;
        }
        return res;
    }
};
