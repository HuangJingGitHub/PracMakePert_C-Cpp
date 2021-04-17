class Solution {
public:
    bool containsDuplicate(vector<int>& nums) {
        if (nums.size() < 2)
            return false;
        
        unordered_set<int> numSet;
        numSet.insert(nums[0]);

        for (int i = 1; i < nums.size(); i++) {
            if (numSet.count(nums[i]) > 0)
                return true;
            numSet.insert(nums[i]);
        }
        return false;
    }
};
