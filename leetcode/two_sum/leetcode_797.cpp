class Solution {
public:
    vector<vector<int>> allPathsSourceTarget(vector<vector<int>>& graph) {
        vector<vector<int>> res;
        vector<int> path;
        backtrack(graph, res, path, 0);
        return res;
    }

    void backtrack(vector<vector<int>>& graph, vector<vector<int>>& res, vector<int>& path, int i) {
        path.push_back(i);
        if (i == graph.size() - 1) {
            res.push_back(path);
            return;
        }

        for (int v : graph[i]) {
            backtrack(graph, res, path, v);
            path.pop_back();
        }
    }
};
