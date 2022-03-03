class Solution {
public:
    vector<double> calcEquation(vector<vector<string>>& equations, vector<double>& values, vector<vector<string>>& queries) {
        unordered_map<string, unordered_map<string, double>> graph;
        int cur = 0;
        for (auto& eq : equations) {
            graph[eq[0]][eq[1]] = values[cur];
            graph[eq[1]][eq[0]] = 1 / values[cur];
            cur++;
        }
           
        vector<double> res;  
        for (auto q : queries) {
            set<string> visited;
            bool found = false;
            double r = dfs(graph, visited, q[0], q[1], found);
            if (found) {
                res.push_back(r);
                graph[q[0]][q[1]] = r;
                graph[q[1]][q[0]] = 1 / r;
            } else {
                res.push_back(-1);
            }
        }
      
        return res;
    }

    double dfs(unordered_map<string, unordered_map<string, double>>& g, set<string> visited, string begin, string end, bool& found) {
        if (g.find(begin) == g.end() || g.find(end) == g.end()) {
            found = false;
            return -1;
        }
        if (visited.find(begin) != visited.end()) {
            found = false;
            return -1;
        }
        if (g[begin].find(end) != g[begin].end()) {
            found = true;
            return g[begin][end];
        }
        visited.insert(begin);
        for (auto it : g[begin]) {
            double r = dfs(g, visited, it.first, end, found);
            if (found) {
                double res = r * it.second;
                return res;
            }
        }
        visited.erase(begin);
        found = false;
        return -1;
    }
};
