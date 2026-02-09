class Solution {
    static constexpr int DIRS[8][2] = {{2, 1}, {1, 2}, {-1, 2}, {-2, 1}, {-2, -1}, {-1, -2}, {1, -2}, {2, -1}};
public:
    double knightProbability(int n, int k, int row, int column) {
        vector memo(k + 1, vector(n, vector<double>(n)));
        auto dfs = [&](this auto& dfs, int k, int i, int j) -> double {
            if (i < 0 || i >= n || j < 0 || j >= n) {
                return 0;
            }
            if (k == 0) {
                return 1;
            }
            double& res = memo[k][i][j]; // 注意这里是引用
            if (res) { // 之前计算过
                return res;
            }
            for (auto& [dx, dy] : DIRS) {
                res += dfs(k - 1, i + dx, j + dy);
            }
            res /= 8;
            return res;
        };
        return dfs(k, row, column);
    }
};
