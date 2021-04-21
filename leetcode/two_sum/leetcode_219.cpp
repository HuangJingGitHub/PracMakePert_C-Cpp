class Solution {
public:
    bool containsNearbyDuplicate(vector<int>& nums, int k) {
        unordered_map<int, int> numToIdx;

        for (int i = 0; i < nums.size(); i++) {
            if (numToIdx.find(nums[i]) == numToIdx.end())
                numToIdx[nums[i]] = i;
            else {
                if (i - numToIdx[nums[i]] <= k)
                    return true;
                else
                    numToIdx[nums[i]] = i;
            }
        }
        return false;
    }
};
