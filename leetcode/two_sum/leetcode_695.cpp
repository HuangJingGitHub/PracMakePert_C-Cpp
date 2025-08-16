class Solution {
    vector<vector<int>> directions{{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
public:
    int maxAreaOfIsland(vector<vector<int>>& grid) {
        int res = 0;
        int m = grid.size(), n = grid[0].size();

        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                if (grid[i][j] == 1) {
                    int area = 0;
                    dfs(grid, i, j, area);
                    res = max(res, area);
                }
            }
        }

        return res;
    }

    void dfs(vector<vector<int>>& grid, int i, int j, int& area) {
        if (grid[i][j] == 0)
            return;

        grid[i][j] = 0;
        area++;
        for (vector<int>& dir : directions) {
            int new_i = i + dir[0], new_j = j + dir[1];
            if (new_i >= 0 && new_i < grid.size()
                && new_j >= 0 && new_j < grid[0].size())
                dfs(grid, new_i, new_j, area);
        }
    }
};
