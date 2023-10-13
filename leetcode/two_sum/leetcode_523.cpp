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


class Solution {
public:
    bool checkSubarraySum(vector<int>& nums, int k) {
        vector<int> sum(nums.size(), 0);
        sum[0] = nums[0];
        for (int i = 1; i < nums.size(); i++)
            sum[i] = nums[i] + sum[i - 1];

        unordered_map<int, int> reminderFirstIdx;
        reminderFirstIdx[0] = -1;
        if (sum[0] % k != 0)
            reminderFirstIdx[sum[0] % k] = 0;
        
        for (int i = 1; i < sum.size(); i++) {
            int reminder = sum[i] % k;
            if (reminderFirstIdx.count(reminder) > 0) {
                if (i - reminderFirstIdx[reminder] > 1)
                    return true;
            }
            else {
                reminderFirstIdx[reminder] = i;
            }
        }
        return false;
    }
};

class Solution {
public:
    bool checkSubarraySum(vector<int>& nums, int k) {
        vector<int> sum(nums.size() + 1, 0);
        for (int i = 1; i < sum.size(); i++)
            sum[i] = nums[i - 1] + sum[i - 1];

        set<int> reminderSet;
        for (int i = 2; i < sum.size(); i++) {
            reminderSet.insert(sum[i - 2] % k);
            int reminder = sum[i] % k;
            if (reminderSet.count(reminder) > 0)
                return true;
        }
        return false;
    }
};
