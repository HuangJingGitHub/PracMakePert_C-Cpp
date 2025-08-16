class Solution {
    vector<vector<int>> dir{{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

public:
    int numEnclaves(vector<vector<int>>& grid) {
        int m = grid.size(), n = grid[0].size(), res = 0;
        for (int col = 0; col < n; col++) {
            dfs(grid, 0, col);
            dfs(grid, m - 1, col);
        }
        for (int row = 1; row < m - 1; row++) {
            dfs(grid, row, 0);
            dfs(grid, row, n - 1);
        }

        for (int row = 1; row < m - 1; row++) {
            for (int col = 1; col < n - 1; col++) {
                if (grid[row][col] == 1)
                    res++;
            }
        }
        return res;
    }

    void dfs(vector<vector<int>>& grid, int row, int col) {
        if (grid[row][col] == 0)
            return;
        
        grid[row][col] = 0;
        for (vector<int>& xy : dir) {
            int new_row = row + xy[0], new_col = col + xy[1];
            if (new_row >= 0 && new_row < grid.size()
                && new_col >= 0 && new_col < grid[0].size()) {
                dfs(grid, new_row, new_col);
            }
        }
    }
};
