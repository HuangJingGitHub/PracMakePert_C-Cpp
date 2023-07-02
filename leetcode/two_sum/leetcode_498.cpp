class Solution {
public:
    vector<int> findDiagonalOrder(vector<vector<int>>& mat) {
        int m = mat.size(), n = mat[0].size(), row = 0, col = 0;
        vector<int> res;

        bool up = true;
        while (row < m && col < n) {
            res.push_back(mat[row][col]);
            if (up == true) {
                if (row  == 0) {
                    up = false;
                    if (col != n - 1) 
                        col++;
                    else 
                        row++;
                }
                else {
                    if (col != n - 1) {
                        row--;
                        col++;
                    }
                    else {
                        row++;
                        up = false;
                    }
                }
            }
            else {
                if (col == 0) {
                    up = true;
                    if (row != m - 1) 
                        row++;
                    else
                        col++;
                }
                else {
                    if (row != m - 1) {
                        row++;
                        col--;
                    }
                    else {
                        col++;
                        up = true;
                    }
                }
            }
        }
        return res;
    }
};
