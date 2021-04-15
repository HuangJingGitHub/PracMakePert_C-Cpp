class Solution {
public:
    vector<vector<int>> combinationSum3(int k, int n) {
        vector<vector<int>> res;
        vector<int> path;

        backtrace(res, path, 1, k, n);
        return res;
    }

    void backtrace(vector<vector<int>>& res, vector<int>& path, int start, int k, int target) {
        if (path.size() == k) {
            if (target == 0) 
                res.push_back(path);
            return;
        }

        for (int i = start; i <= 9; i++) {
            if (i > target)
                break;
            path.push_back(i);
            backtrace(res, path, i + 1, k, target - i);
            path.pop_back();
        }
    }
};
