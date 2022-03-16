// 插孔法做排列，没有用回溯和递归
class Solution {
public:
    vector<vector<int>> permute(vector<int>& nums) {
        vector<vector<int>> res;
        if (nums.size() == 1 || nums.size() == 0) {
            res.push_back(nums);
            return res;
        }

        vector<int> initVec{nums[0]};
        res.push_back(initVec);
        int currentSize;
        for(int i = 1; i < nums.size(); i++) {
            currentSize = res.size();
            for (int j = 0; j < i; j++) {
                for (int k = 0; k < currentSize; k++) {
                    res.push_back(res[k]);
                }
                for(int m = 0; m < currentSize; m++) {
                    res[res.size() - m - 1].insert(res[res.size() - m - 1].begin() + j + 1, nums[i]);
                }
            }
            for (int n = 0; n < currentSize; n++)
                res[n].insert(res[n].begin(), nums[i]);
        }
        return res;
    }
};


// This is the backtrack algorithm + dfs. See the discussion for details.
class Solution {
public:
    vector<vector<int>> permute(vector<int>& nums) {
        int len = nums.size();
        vector<vector<int>> res;
        vector<bool> used(len, false);
        vector<int> path;

        dfs(nums, path, used, res);
        return res;
    }

private:
    void dfs(vector<int> nums, vector<int> path, vector<bool> used, vector<vector<int>>& res) {
            if (path.size() == nums.size()) {
                res.push_back(path);
                return;
            }

            for (int i = 0; i < len; i++) {
                if (used[i] == true)
                    continue;
                
                path.push_back(nums[i]);
                used[i] = true;
                dfs(nums, path, used, res);
                used[i] = false;
                path.pop_back();
                }
            }
        }
};
