// good use of set
class Solution {
public:
    bool containsNearbyAlmostDuplicate(vector<int>& nums, int k, int t) {
        set<long int> kSet;
        
        for (int i = 0; i < nums.size(); i++) {
            auto lb = kSet.lower_bound((long)nums[i] - t);
            if (lb != kSet.end() && *lb <= (long)nums[i] + t)
                return true;

            kSet.insert(nums[i]);
            if (i >= k)
                kSet.erase(nums[i - k]);
        }
        return false;
    }
};
