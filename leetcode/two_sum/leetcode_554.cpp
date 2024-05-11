class Solution {
public:
    int leastBricks(vector<vector<int>>& wall) {
        unordered_map<int, int> edge_times;

        int rows = wall.size();
        for (int i = 0; i < rows; i++) {
            int cur_width = 0;
            for (int j = 0; j < wall[i].size() - 1; j++) {
                cur_width += wall[i][j];
                edge_times[cur_width]++;
            }
        }

        int max_edge_times = 0;
        for (auto it = edge_times.begin(); it != edge_times.end(); it++)
            max_edge_times = max(max_edge_times, it->second);

        return rows - max_edge_times;
    }
};
