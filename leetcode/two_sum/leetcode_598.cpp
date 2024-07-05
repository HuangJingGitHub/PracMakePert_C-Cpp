class Solution {
public:
    int maxCount(int m, int n, vector<vector<int>>& ops) {
        if (ops.size() == 0)
            return m * n;

        int row_num = 1e5, col_num = 1e5;
        for (auto& ops_i : ops) {
            row_num = min(row_num, ops_i[0]);
            col_num = min(col_num, ops_i[1]);
        }  
        return row_num * col_num;
    }
};
