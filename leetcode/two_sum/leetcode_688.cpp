class Solution {
    static constexpr int DIRS[8][2] = {{2, 1}, {1, 2}, {-1, 2}, {-2, 1}, {-2, -1}, {-1, -2}, {1, -2}, {2, -1}};

public:
    double knightProbability(int n, int k, int row, int column) {
        vector memo(k + 1, vector(n, vector<double>(n)));
        return dfs(k, row, column, n, memo);
    }

    double dfs(int k, int row, int col, int n, vector<vector<vector<double>>>& memo) {
        if (row < 0 || row >= n || col < 0 || col >= n)
            return 0;
        if (k == 0)
            return 1;
        
        double& res = memo[k][row][col];
        if (res)
            return res;
        
        for (auto& [dx, dy] : DIRS)
            res += dfs(k - 1, row + dx, col + dy, n, memo);
        res /= 8;
        return res;
    }
};
