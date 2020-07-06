class Solution {
public:
    vector<vector<int>> res;
    vector<vector<int>> subsetsWithDup(vector<int>& nums) {
        sort(nums.begin(), nums.end());   // First sort
        vector<int> path;
        trackback(0, nums, path);
        return res;
    }

    void trackback(int start, vector<int>& nums, vector<int> &path)
    {
        res.push_back(path);

        for (int i = start; i < nums.size(); i++){
            if (i != start && nums[i] == nums[i-1])  // The essence is i != start
                continue;
            path.push_back(nums[i]);
            trackback(i+1, nums, path);
            path.pop_back();
        }
    }
};
