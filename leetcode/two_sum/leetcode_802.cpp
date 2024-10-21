class Solution {
public:
    vector<int> eventualSafeNodes(vector<vector<int>>& graph) {
        vector<int> res;
        vector<bool> visited(graph.size(), false);
        for (int i = 0; i < graph.size(); i++) {
            if (dfs(graph, visited, i, res) == true)
                res.push_back(i);
        }
        return res;
    }

    bool dfs(vector<vector<int>>& graph, vector<bool>& visited, int i, vector<int>& res) {
        if (visited[i] == true)
            return false;

        visited[i] = true;
        for (int j : graph[i]) {
            if (dfs(graph, visited, j, res) == false)
                return false;
        }
        return true;
    }
};
