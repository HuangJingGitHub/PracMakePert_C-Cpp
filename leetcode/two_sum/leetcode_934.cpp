class Solution {
public:
    int shortestBridge(vector<vector<int>>& grid) {
        int res = 0;
        vector<vector<int>> directions = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};
        queue<vector<int>> boundary;
        findFirstIsland(grid, boundary);

        while (boundary.empty() == false) {
            res++;
            for (int boundary_num = boundary.size(); boundary_num > 0; boundary_num--) {
                vector<int> boundary_pos = boundary.front();
                boundary.pop();
                
                for (int i = 0; i < 4; i++) {
                    int new_row = boundary_pos[0] + directions[i][0],
                        new_col = boundary_pos[1] + directions[i][1];
                    if (new_row < 0 || new_row >= grid.size() || new_col < 0 || new_col >= grid[0].size())
                        continue;

                    if (grid[new_row][new_col] == 1)
                        return res;
                    else if (grid[new_row][new_col] == 0) {
                        grid[new_row][new_col] = 2;
                        boundary.push({new_row, new_col});
                    }
                }
            }
        }
        return res;
    }

    void findFirstIsland(vector<vector<int>>& grid, queue<vector<int>>& boundary) {
        for (int i = 0; i < grid.size(); i++)
            for (int j = 0; j < grid[0].size(); j++) {
                if (grid[i][j] == 1) {
                    dfs(grid, i, j, boundary);
                    return;
                }
            }
    }

    void dfs(vector<vector<int>>& grid, int row, int col, queue<vector<int>>& boundary) {
        if (row < 0 || row >= grid.size() || col < 0 || col >= grid[0].size())
            return;
        if (grid[row][col] == 2)
            return;
        if (grid[row][col] == 0) {
            boundary.push({row, col});
            grid[row][col] = 2;
            return;
        }
        // if grid[row][col] == 1
        grid[row][col] = 2;
        dfs(grid, row + 1, col, boundary);
        dfs(grid, row - 1, col, boundary);
        dfs(grid, row, col - 1, boundary);
        dfs(grid, row, col + 1, boundary);
    }
};
