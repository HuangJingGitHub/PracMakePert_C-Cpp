class Solution {
public:
    bool canPartitionKSubsets(vector<int>& nums, int k) {
        int sum = 0;
        for (int& n : nums)
            sum += n;
        if (sum % k != 0)
            return false;
        int target = sum / k;
        
        sort(nums.begin(), nums.end(), greater<int>());
        vector<int> buckets(k, 0);
        return backtrack(nums, 0, buckets, k, target);
    }

    bool backtrack(vector<int>& nums, int idx, vector<int>& buckets, int k, int target) {
        if (idx == nums.size())
            return true;
        for (int i = 0; i < k; i++) {
            if (i > 0 && buckets[i] == buckets[i - 1])
                continue;
            if (buckets[i] + nums[idx] > target)
                continue;
            buckets[i] += nums[idx];
            if (backtrack(nums, idx + 1, buckets, k, target))
                return true;
            buckets[i] -= nums[idx];
        }
        return false;
    }
};
