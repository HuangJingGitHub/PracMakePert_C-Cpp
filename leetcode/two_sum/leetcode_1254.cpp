// classical BFS
class Solution {
public:
    int closedIsland(vector<vector<int>>& grid) {
        int res = 0;
        if (grid.size() == 0 || grid[0].size() == 0)
            return res;

        for (int i = 0; i < grid.size(); i++)
            for (int j = 0; j < grid[0].size(); j++){
                if (grid[i][j] == 0){
                    if (!DFS_ReachBorder(grid, i, j))
                        res++;
                }
            }
        
        return res;
    }

    bool DFS_ReachBorder(vector<vector<int>>& grid, int row, int col){
        if (row < 0 || row >= grid.size() || col < 0 || col >= grid[0].size())
            return true;
        if (grid[row][col] != 0)
            return false;
        
        grid[row][col] = -1;  // represent visited land

        bool left = DFS_ReachBorder(grid, row, col - 1);
        bool right = DFS_ReachBorder(grid, row, col + 1);
        bool up = DFS_ReachBorder(grid, row - 1, col);
        bool down = DFS_ReachBorder(grid, row + 1, col);
        
        return left || right || up || down;
    }
};

    /* A subtle detail is that all search in all 4 directions need be finished before 
          return the bool value using or operator. The follwing code is incorrect as it may
          bring out a larger result:
            return DFS_ReachBorder(grid, row, col - 1) || DFS_ReachBorder(grid, row, col + 1)
               || DFS_ReachBorder(grid, row - 1, col)
               || DFS_ReachBorder(grid, row + 1, col);
        
        The reason is that a land needs to run over all the lands in one island before returning
        so that the remarking of all the adjacent islands can be finished. 
        Otherwise it may create new closed island. See the following example:
          [[1, 1, 1, 1, 1]
           [1, 1, 0, 1, 1],
           [0, 0, 0, 0, 1],
           [1, 1, 1, 1, 1]]  
        Run the direct return code, the grids will change as:
          [[1, 1, 1, 1, 1]
           [1, 1, -1, 1, 1], 
           [0, 0, 0, 0, 1],
           [1, 1, 1, 1, 1]]  
           ---> ...
          [[1, 1, 1, 1, 1]
           [1, 1, -1, 1, 1],
           [-1, -1, -1, 0, 1],
           [1, 1, 1, 1, 1]]
        where a closed island is created by the program and result in an incorrect answer. */

