class Solution {
public:
    int findPairs(vector<int>& nums, int k) {
        set<int> target_num, smaller_pair_num;

        for (int i = 0; i < nums.size(); i++) {
            if (target_num.count(nums[i] + k))
                smaller_pair_num.insert(nums[i]);
            if (target_num.count(nums[i] - k))
                smaller_pair_num.insert(nums[i] - k);
            target_num.insert(nums[i]);
        }
        return smaller_pair_num.size();
    }
};
