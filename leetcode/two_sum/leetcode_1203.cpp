class Solution {
public:
    vector<int> sortItems(int n, int m, vector<int>& group, vector<vector<int>>& beforeItems) {
        for (int& g : group)
            if (g == -1)
                g = m++;

        vector<int>  group_indegree(m), item_indegree(n);
        vector<vector<int>> group_adj(m), item_adj(n);
        for (int i = 0; i < n; i++) {
            int cur_group = group[i];
            for (int b : beforeItems[i]) {
                int before_group = group[b];
                if (before_group != cur_group) {
                    group_adj[before_group].push_back(cur_group);
                    group_indegree[cur_group]++;
                }
                item_adj[b].push_back(i);
                item_indegree[i]++;
            }
        }

        vector<int> group_list = topologicalSort(group_adj, group_indegree, m);
        if (group_list.size() == 0)
            return {};
        vector<int> item_list = topologicalSort(item_adj, item_indegree, n);
        if (item_list.size() == 0)
            return {};
        
        vector<vector<int>> group_to_item(m);
        for (int i : item_list) 
            group_to_item[group[i]].push_back(i);
        
        vector<int> res;
        for (int g : group_list)
            res.insert(res.end(), group_to_item[g].begin(), group_to_item[g].end());
        return res;
    }

    vector<int> topologicalSort(vector<vector<int>>& adj, vector<int>& ind, int n) {
        queue<int> q; 
        vector<int> res;
        for (int i = 0; i < n; i++)
            if (ind[i] == 0)
                q.push(i);
        
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            res.push_back(u);
            for (int v : adj[u]) {
                ind[v]--;
                if (ind[v] == 0)
                    q.push(v);
            }
        }
        return res.size() == n ? res : vector<int>{};
    }
};
