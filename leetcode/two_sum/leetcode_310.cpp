class Solution {
public:
    vector<int> findMinHeightTrees(int n, vector<vector<int>>& edges) {
        vector<int> res, leaf;
        vector<unordered_set<int>> indegree(n);

        for (auto& edge : edges) {
            indegree[edge[0]].insert(edge[1]);
            indegree[edge[1]].insert(edge[0]);
        }

        while (true) {
            leaf.clear();
            for (int idx = 0 ; idx < n; idx++) {
                if (indegree[idx].size() == 1)
                    leaf.push_back(idx);
            }
            
            if (leaf.size() == 0)
                break;
            else {
                for (int& nodeIdx : leaf) {
                    int pair_node = *(indegree[nodeIdx].begin());
                    indegree[nodeIdx].clear();
                    indegree[pair_node].erase(nodeIdx);
                }
                res.clear();
                for (int idx = 0; idx < n; idx++)
                    if (indegree[idx].size() == 1)
                        res.push_back(idx);
            }
        }
        return res;
    }
};
