class Solution {
public:
    int countServers(vector<vector<int>>& grid) {
        if (grid.empty() || grid[0].empty())
            return 0;
            
        int rows = grid.size(), cols = grid[0].size(), res = 0;
        vector<int> rowSum(rows, 0), colSum(cols, 0);
        for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++) {
                if (grid[r][c] == 1) {
                    rowSum[r]++;
                    colSum[c]++;
                }
            }
        
        for (int sum : rowSum)
            if (sum > 1)
                res += sum;
        for (int c = 0; c < cols; c++)
            if (colSum[c] > 1)
                for (int r = 0; r < rows; r++) 
                    if (grid[r][c] == 1 && rowSum[r] == 1)
                        res++;
        return res;
    }
};
