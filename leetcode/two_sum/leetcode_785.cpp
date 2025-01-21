class Solution {
public:
    bool isBipartite(vector<vector<int>>& graph) {
        int n = graph.size();
        vector<int> label(n, 0);

        for (int i = 0; i < n; i++) {
            if (label[i] == 0) {
                queue<int> level;
                level.push(i);
                label[i] = 1;

                while (level.empty() == false) {
                    int u = level.front();
                    level.pop();

                    for (int v : graph[u]) {
                        if (label[v] == label[u])
                            return false;
                        else if (label[v] == 0) {
                            label[v] = -label[u];
                            level.push(v);
                        }
                    }
                }
            }
        }
        return true;
    }
};
