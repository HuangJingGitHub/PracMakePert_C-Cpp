class Solution {
public:
    vector<int> findMinHeightTrees(int n, vector<vector<int>>& edges) {
        vector<vector<int>> graph(n);
        vector<int> res;

        if (n == 1) {
            res.push_back(0);
            return res;
        }

        vector<int> outdegree(n, 0);
        for (int i = 0; i < edges.size(); i++) {
            graph[edges[i][0]].push_back(edges[i][1]);
            graph[edges[i][1]].push_back(edges[i][0]);
            outdegree[edges[i][0]]++;
            outdegree[edges[i][1]]++;
        }

        queue<int> leaf;
        for (int i = 0; i < n; i++)
            if (outdegree[i] == 1)
                leaf.push(i);
        
        while (!leaf.empty()) {
            int leafNum = leaf.size();
            vector<int> temp;
            for (int i = 0; i < leafNum; i++) {
                int curNode = leaf.front();
                leaf.pop();
                temp.push_back(curNode);
                int pairNode = graph[curNode][0];
                outdegree[pairNode]--;
                auto itr = find(graph[pairNode].begin(), graph[pairNode].end(), curNode);
                graph[pairNode].erase(itr);
                if (outdegree[pairNode] == 1)
                    leaf.push(pairNode);
            }
            res = temp;
        }
        return res;
    }
};
