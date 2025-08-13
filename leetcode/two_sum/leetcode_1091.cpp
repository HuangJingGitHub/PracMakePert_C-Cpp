class Solution {
public:
    int shortestPathBinaryMatrix(vector<vector<int>>& grid) {
        vector<int> direction_x{-1, 0, 1}, direction_y{-1, 0, 1};
        if (grid[0][0] == 1)
            return -1;
        
        int n = grid.size(), res = 0;
        if (n == 1)
            return 1;
        
        vector<vector<bool>> visited(n, vector<bool>(n, false));
        queue<vector<int>> pos;
        pos.push({0, 0});
    }
};
