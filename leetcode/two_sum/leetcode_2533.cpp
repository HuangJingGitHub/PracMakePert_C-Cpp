class Solution {
public:
    vector<vector<int>> combinationSum2(vector<int>& candidates, int target) {
        vector<vector<int>> res;
        vector<int> path;
        sort(candidates.begin(), candidates.end());

        backtrack(0, candidates, path, res, 0, target);
        return res;
    }

    void backtrack(int curIdx, vector<int>& candidates, vector<int>& path, vector<vector<int>>& res, int curSum, int& target) {
        if (curSum == target) {
            res.push_back(path);
            return;
        }

        for (int i = curIdx; i < candidates.size(); i++) {
            if (i > curIdx && candidates[i] == candidates[i - 1])
                continue;
            if (curSum > target)
                break;

            path.push_back(candidates[i]);
            backtrack(i + 1, candidates, path, res, curSum + candidates[i], target);
            path.pop_back();
        }
    }
};
