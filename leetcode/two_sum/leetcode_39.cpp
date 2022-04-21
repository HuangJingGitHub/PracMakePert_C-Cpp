// See the reference solution for backtrack method.
class Solution {
private:
    vector<int> candidates;
    vector<vector<int>> res;
    vector<int> path;

public:
    void backtrack(int start, int target) {
        if (target == 0) {
            res.push_back(path);
            return;
        }
        for (int i = start; i < candidates.size() && target - candidates[i] >= 0; i++) {
            path.push_back(candidates[i]);
            backtrack(i, target - candidates[i]);
            path.pop_back();
        }
    }

    vector<vector<int>> combinationSum(vector<int>& candidates, int target) {
        sort(candidates.begin(), candidates.end());
        this->candidates = candidates;
        backtrack(0, target);
        return res;
    }
};


// Personal sol, basically the same as above
class Solution {
private:
    vector<vector<int>> res;
    vector<int> path;

public:
    vector<vector<int>> combinationSum(vector<int>& candidates, int target) {
        sort(candidates.begin(), candidates.end());
        backtrack(0, target, candidates);
        return res;
    }

    void backtrack(int startIdx, int target, vector<int>& candidates) {
        if (target == 0) {
            res.push_back(path);
            return;
        }

        for (int i = startIdx; i < candidates.size(); i++) {
            if (candidates[i] <= target) {
                path.push_back(candidates[i]);
                backtrack(i, target - candidates[i], candidates);
                path.pop_back();
            }
            else
                return;
        }
    }
};
