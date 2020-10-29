// Tarjan algorithm
class Solution {
public:
    vector<vector<int>> res;
    vector<vector<int>> adjacency;
    vector<int> idx;
    vector<int> depth; // Nodes in a circle are at the same depth.
    vector<bool> visited;
    int time = 1;

    vector<vector<int>> criticalConnections(int n, vector<vector<int>>& connections) {
        adjacency = vector<vector<int>>(n);
        idx = vector<int>(n, 0);
        depth = vector<int>(n, 0);

        for (int i = 0; i < connections.size(); i++){
            adjacency[connections[i][0]].push_back(connections[i][1]);
            adjacency[connections[i][1]].push_back(connections[i][0]);
        }

        for (int i = 0; i < n; i++){
            if (idx[i] == 0)
                DFS(-1, i);
        }

        return res;
    }

    void DFS(int parent, int i){
        idx[i] = time;
        depth[i] = time++;

        for (int x : adjacency[i]){
            if (idx[x] == 0)
                DFS(i, x);
            if (x != parent)
                depth[i] = min(depth[i], depth[x]);
            if (depth[x] > idx[i])
                res.push_back({i, x});
        }

    }
};
