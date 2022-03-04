class Solution {
public:
    int classCnt_ = 0;
    unordered_map<int, vector<int>> idxMap_;
    Solution(vector<int>& nums) {
        for (int i = 0; i < nums.size(); i++) {
            idxMap_[nums[i]].push_back(i);
        }
    }
    
    int pick(int target) {
        if (classCnt_ == INT_MAX)
            classCnt_ = 0;

        return idxMap_[target][classCnt_++ % idxMap_[target].size()];
    }
};

/**
 * Your Solution object will be instantiated and called as such:
 * Solution* obj = new Solution(nums);
 * int param_1 = obj->pick(target);
 */
