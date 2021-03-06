// also good example for traceback, review last Problem 77
class Solution {
public:
    vector<vector<int>> res;

    void traceback(vector<int> nums, int k, int begin, vector<int>& path)
    {
        if (path.size() == k){
            res.push_back(path);
            return;
        }

        for (int i = begin; i < nums.size()-(k-path.size())+1; i++){
            path.push_back(nums[i]);
            traceback(nums, k, i+1, path);
            path.pop_back();
        }
    }

    void findCombinations(vector<int>& nums, int k)
    {
        if (k < 0 || k > nums.size())
            return;
        
        vector<int> path;
        traceback(nums, k, 0, path);
    }

    vector<vector<int>> subsets(vector<int>& nums) 
    {
        for (int i = 0; i <= nums.size(); i++)
            findCombinations(nums, i);
        return res;
    }
};

// update: more compact code
class Solution {
public:
    vector<vector<int>> res;
    void trackback(int start, vector<int>& nums, vector<int>& path){
        res.push_back(path);

        for (int i = start; i < nums.size(); i++){
            path.push_back(nums[i]);
            trackback(i+1, nums, path);
            path.pop_back();
        }
    }

    vector<vector<int>> subsets(vector<int>& nums) 
    {
        vector<int> path;
        trackback(0, nums, path);
        return res;
    }
};
