// classical DFS
class Solution {
public:
    int numIslands(vector<vector<char>>& grid) {
        int res = 0;
        if (grid.size() == 0 || grid[0].size() == 0)
            return res;

        for (int i = 0; i < grid.size(); i++)
            for (int j = 0; j < grid[0].size(); j++){
                if (grid[i][j] == '1')
                    if (DFS_ReacheWater(grid, i, j))
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

        bool left = DFS_ReacheWater(grid, row, col - 1);
        bool right = DFS_ReacheWater(grid, row, col + 1);
        bool up = DFS_ReacheWater(grid, row - 1, col);
        bool down = DFS_ReacheWater(grid, row + 1, col);

        return left && right && up && down;
    }
};
