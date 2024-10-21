class Solution {
public:
    vector<int> eventualSafeNodes(vector<vector<int>>& graph) {
        vector<int> res;
        vector<bool> visited(graph.size(), false), is_eventual_safe(graph.size(), false);

        for (int i = 0; i < graph.size(); i++) 
            if (dfs(graph, visited, is_eventual_safe, i) == true)
                res.push_back(i);
        
        return res;
    }

    bool dfs(vector<vector<int>>& graph, vector<bool>& visited, vector<bool>& is_eventual_safe, int i) {
        if (visited[i] == true) 
            return is_eventual_safe[i];

        visited[i] = true;
        for (int j : graph[i]) 
            if (dfs(graph, visited, is_eventual_safe, j) == false) 
                return false;
        
        is_eventual_safe[i] = true;
        return true;
    }
};
