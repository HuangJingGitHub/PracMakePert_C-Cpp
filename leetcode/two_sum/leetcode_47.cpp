class Solution {
public:
    vector<vector<int>> permuteUnique(vector<int>& nums) {
        int len = nums.size();
        vector<vector<int>> res;

        if (len == 0 || len == 1)
        {
            res.push_back(nums);
            return res;
        }

        vector<bool> used(len, false);
        vector<int> path;

        sort(nums.begin(), nums.end());   // First sort.
        dfs(nums, len, 0, path, used, res);
        return res;
    }

private:
    void dfs(vector<int> nums, int len, int depth, vector<int> path, vector<bool> used, 
            vector<vector<int>>& res){
            if (depth == len){
                res.push_back(path);
                return;
            }

            for (int i = 0; i < len; i++){
                if (!used[i]){
                    if (i > 0 && nums[i-1] == nums[i] && !used[i-1]) // Add judgement.
                        continue;

                    path.push_back(nums[i]);
                    used[i] = true;

                    dfs(nums, len, depth + 1, path, used, res);
                    used[i] = false;
                    path.pop_back();
                }
            }
        }        
};
