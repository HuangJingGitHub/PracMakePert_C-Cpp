class Solution {
public:
    vector<vector<int>> directions{{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
    int rowNum, colNum;
    
    int longestIncreasingPath(vector<vector<int>>& matrix) {
        int res = 0;
        rowNum = matrix.size();
        colNum = matrix[0].size();
        vector<vector<int>> memo(rowNum, vector<int>(colNum, 0));
        
        for (int row = 0; row < rowNum; row++)
            for (int col = 0; col < colNum; col++)
                res = max(res, dfs(matrix, memo, row, col));
        return res;
    }

    int dfs(vector<vector<int>>& matrix, vector<vector<int>>& memo, int row, int col) {
        if (memo[row][col] != 0)
            return memo[row][col];
        
        memo[row][col]++;
        for (int dir = 0; dir < 4; dir++) {
            int newRow = row + directions[dir][0], newCol = col + directions[dir][1];
            if (newRow >= 0 && newRow < rowNum && newCol >= 0 && newCol < colNum && matrix[row][col] > matrix[newRow][newCol])
                memo[row][col] = max(memo[row][col], dfs(matrix, memo, newRow, newCol) + 1);
        }
        return memo[row][col];
    }
};
