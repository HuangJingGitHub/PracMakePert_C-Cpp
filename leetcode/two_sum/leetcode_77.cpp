// backtrack --- should be dfs
class Solution {
public:
    vector<vector<int>> res;
    
    void backtrack(int n, int k, int begin, vector<int> pre) {
        if (pre.size() == k) {
            res.push_back(pre);
            return;
        }

        for (int i = begin; i <= n; i++){  // Optimize: for (int i = begin; i <= n-k+pre.size()+1; i++)
            pre.push_back(i);
            backtrack(n, k, i+1, pre);
            pre.pop_back();
        }
    }

    vector<vector<int>> combine(int n, int k) {
        if (n <= 0 || k <= 0 || n < k)
            return res;
        vector<int> path;
        backtrack(n, k, 1, path);
        return res;
    }
};


class Solution {
public:
    vector<vector<int>> combine(int n, int k) {
        vector<vector<int>> res;
        vector<int> path;
        dfs(1, n, k, path, res);
        return res;
    }

    void dfs(int curNum, int n, int k, vector<int>& path, vector<vector<int>>& res) {
        if (path.size() == k) {
            res.push_back(path);
            return;
        }

        for (int i = curNum; i <= n; i++) {
            path.push_back(i);
            dfs(i + 1, n, k, path, res);
            path.pop_back();
        }
    }
};
