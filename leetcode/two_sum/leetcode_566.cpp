class Solution {
public:
    vector<vector<int>> matrixReshape(vector<vector<int>>& mat, int r, int c) {
        int m = mat.size(), n = mat.front().size();
        if (m * n != r * c)
            return mat;
        
        vector<vector<int>> res(r, vector<int>(c));
        for (int row = 0; row < m; row++)
            for (int col = 0; col < n; col++) {
                int idx = row * n + col;
                res[idx / c][idx % c] = mat[row][col];
            }
        return res;
    }
};
