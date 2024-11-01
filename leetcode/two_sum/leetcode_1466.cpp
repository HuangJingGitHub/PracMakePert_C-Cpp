class Solution {
public:
    int minReorder(int n, vector<vector<int>>& connections) {
        int res = 0;
        vector<set<int>> adj(n), road(n);
        vector<bool> visited(n, false);
        for (auto con : connections) {
            road[con[0]].insert(con[1]);
            adj[con[0]].insert(con[1]);
            adj[con[1]].insert(con[0]);
        }

        queue<int> city_level;
        city_level.push(0);
        while (city_level.empty() == false) {
            int cur_city = city_level.front();
            city_level.pop();
            visited[cur_city] = true;
            for (int near_city : adj[cur_city]) {
                if (visited[near_city] == true)
                    continue;
                if (road[cur_city].count(near_city) > 0)
                    res++;
                city_level.push(near_city);
            }
        }
        return res;
    }
};
