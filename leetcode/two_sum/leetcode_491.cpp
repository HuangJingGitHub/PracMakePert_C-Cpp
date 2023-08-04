class Solution {
public:
    vector<vector<int>> result;
    vector<int> path;

    void backtrack(vector<int>& nums, int startIdx) {
        if (path.size() > 1)
            result.push_back(path);
        if (startIdx > 0 && nums[startIdx] == nums[startIdx - 1])
            return;
        
        for (int i = startIdx; i < nums.size(); i++) {
            if ((path.empty() == false && path.back() > nums[i]) 
                || uniqueSet.find(nums[i]) != uniqueSet.end())
                continue;
            
            
            path.push_back(nums[i]);
            backtrack(nums, i + 1);
            path.pop_back();
        }
    }

    vector<vector<int>> findSubsequences(vector<int>& nums) {
        result.clear();
        path.clear();
        backtrack(nums, 0);
        return result;
    }
};
