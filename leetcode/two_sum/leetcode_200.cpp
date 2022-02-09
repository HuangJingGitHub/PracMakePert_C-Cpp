// classical DFS
class Solution {
public:
    int numIslands(vector<vector<char>>& grid) {
        int res = 0;
        if (grid.size() == 0 || grid[0].size() == 0)
            return res;

        for (int i = 0; i < grid.size(); i++)
            for (int j = 0; j < grid[0].size(); j++) {
                if (grid[i][j] == '1' && DFS_ReacheWater(grid, i, j))
                    res++;
            }
        return res;
    }

    bool DFS_ReacheWater(vector<vector<char>>& grid, int row, int col){
        if (row < 0 || row >= grid.size() || col < 0 || col >= grid[0].size())
            return true;
        if (grid[row][col] != '1')  // is water or visited land
            return true;
        grid[row][col] = '2';  // mark visited land

        bool left = DFS_ReacheWater(grid, row, col - 1),
             right = DFS_ReacheWater(grid, row, col + 1),
             up = DFS_ReacheWater(grid, row - 1, col),
             down = DFS_ReacheWater(grid, row + 1, col);

        return left && right && up && down;
    }
};


// classical DFS, lighter implementation
class Solution {
public:
    vector<int> dx{-1, 0, 0, 1}, dy{0, -1, 1, 0};
    int m, n;

    int numIslands(vector<vector<char>>& grid) {
        int res = 0;
        m = grid.size(), n = grid[0].size();

        for (int row = 0; row < m; row++) {
            for (int col = 0; col < n; col++)
                if (grid[row][col] == '1') {
                    dfs(grid, row, col);
                    res++;
                }
        }
        return res;
    }
    
    void dfs(vector<vector<char>>& grid, int row, int col) {
        grid[row][col] = '0';

        for (int i = 0; i < 4; i++) {
            int newRow = row + dx[i], newCol = col + dy[i];
            if (0 <= newRow && newRow < m && 0 <= newCol && newCol < n) {
                if (grid[newRow][newCol] == '1')
                    dfs(grid, newRow, newCol);
            }
        }    
    }
};
