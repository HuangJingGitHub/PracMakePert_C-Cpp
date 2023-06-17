class Solution {
public:
    int islandPerimeter(vector<vector<int>>& grid) {
        int res = 0, row_num = grid.size(), col_num = grid[0].size();
        for (int row = 0; row < row_num; row++)
            for (int col = 0; col < col_num; col++) {
                if (grid[row][col] == 1)
                    res += calculatePerimeterofCell(grid, row, col);
            }
        return res;
    }

    int calculatePerimeterofCell(vector<vector<int>>& grid, int row, int col) {
        int res = 0;
        // up
        if (row == 0 || (row > 0 && grid[row - 1][col] == 0))
            res++;
        // down
        if (row == grid.size() - 1 || (row < grid.size() - 1 && grid[row + 1][col] == 0))
            res++;
        // left
        if (col == 0 || (col > 0 && grid[row][col - 1] == 0))
            res++;
        // right
        if (col == grid[0].size() - 1 || (col < grid[0].size() - 1 && grid[row][col + 1] == 0))
            res++;
        return res;
    }
};
