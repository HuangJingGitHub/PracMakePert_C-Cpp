class Solution {
public:
    vector<int> shortestAlternatingPaths(int n, vector<vector<int>>& redEdges, vector<vector<int>>& blueEdges) {
        // ONLY WORKS FOR ACYLIC GRAPH
        // red_dist: shortest alternating path length ending with red dege
        // blue_dist: shortest alternating path length ending with blue dege
        vector<int> res(n, -1), red_dist(n, -1), blue_dist(n, -1), visited(n, false); 
        vector<set<int>> red_adj(n), blue_adj(n), adj(n);
        for (auto edge : redEdges) {
            int u = edge[0], v = edge[1];
            red_adj[u].insert(v);
            adj[u].insert(v);
        }
        for (auto edge : blueEdges) {
            int u = edge[0], v =edge[1];
            blue_adj[u].insert(v);
            adj[u].insert(v);
        }

        res[0] = 0;
        red_dist[0] = 0;
        blue_dist[0] = 0;
        queue<int> level;
        level.push(0);
        while (level.empty() == false) {
            int u = level.front();
            level.pop();
            visited[u] = true;
            for (int v : adj[u]) {
                if (red_adj[u].count(v) && blue_dist[u] != -1)
                    red_dist[v] = red_dist[v] == -1 ? blue_dist[u] + 1 : min(red_dist[v], blue_dist[u] + 1);
                if (blue_adj[u].count(v) && red_dist[u] != -1)
                    blue_dist[v] = blue_dist[v] == -1 ? red_dist[u] + 1 : min(blue_dist[v], red_dist[u] + 1);

                if (red_dist[v] != -1 && blue_dist[v] != -1)
                    res[v] = min(red_dist[v], blue_dist[v]);
                if (red_dist[v] != -1 && blue_dist[v] == -1)
                    res[v] = red_dist[v];
                if (red_dist[v] == -1 && blue_dist[v] != -1)
                    res[v] = blue_dist[v];
                
                if (visited[v] == false)
                    level.push(v);
            }
        }
        return res;
    }
};
