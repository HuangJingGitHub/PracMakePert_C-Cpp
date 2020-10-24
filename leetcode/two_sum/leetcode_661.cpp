// Not hard problem. Bulky solution, not that elegant. Consider implementing kernel operator.
class Solution {
public:
    vector<vector<int>> imageSmoother(vector<vector<int>>& M) {
        int row = M.size(), col = M[0].size();
        vector<vector<int>> res(row, vector<int>(col, 0));
        // process vertices
        if (row == 1 && col == 1)
            return M;
        else if (row == 1 && col > 1){
            res[0][0] = (M[0][0] + M[0][1]) / 2;
            res[0][col - 1] = (M[0][col - 2] + M[0][col - 1]) / 2;
            for (int i = 1; i <= col - 2; i++)
                res[0][i] = (M[0][i - 1] + M[0][i] + M[0][i + 1]) / 3;
            return res;
        }
        else if (row >= 2 && col > 1){
            res[0][0] = (M[0][0] + M[0][1] + M[1][0] + M[1][1]) / 4;
            res[0][col - 1] = (M[0][col - 1] + M[0][col - 2] + M[1][col - 2] + M[1][col - 1]) / 4;
            res[row - 1][0] = (M[row  - 1][0] + M[row - 1][1] + M[row - 2][0] + M[row - 2][1]) / 4;
            res[row - 1][col - 1] = (M[row - 1][col - 1] + M[row - 1][col - 2] + M[row - 2][col - 1] + M[row - 2][col - 2]) / 4;
        }
        else if (row >= 2 && col == 1){
            res[0][0] = (M[0][0] + M[1][0]) / 2;
            res[row - 1][0] = (M[row - 1][0] + M[row - 2][0]) / 2;
            for (int i = 1; i <= row - 2; i++)
                res[i][0] = (M[i][0] + M[i - 1][0] + M[i + 1][0]) / 3;
            return res;
        }
            
        for (int i = 0; i < row; i++)
            for (int j = 0; j < col; j++){
                if (i == 0){
                    if (j == 0 || j == col - 1)
                        continue;
                    res[0][j] = (M[0][j] + M[0][j - 1] + M[1][j - 1] + M[1][j] + M[1][j + 1] + M[0][j + 1]) / 6;
                }
                else if (i == row - 1){
                    if (j == 0 || j == col - 1)
                        continue;
                    res[row - 1][j] = (M[row - 1][j] + M[row - 1][j - 1] + M[row - 2][j - 1] + M[row - 2][j] + M[row - 2][j + 1] + M[row - 1][j + 1]) / 6;
                }
                else if (j == 0){
                    if (i == 0 || i == row - 1)
                        continue;
                    res[i][0] = (M[i][0] + M[i - 1][0] + M[i - 1][1] + M[i][1] + M[i + 1][1] + M[i + 1][0]) / 6;
                }
                else if (j == col - 1){
                    if (i == 0 || i == row - 1)
                        continue;
                    res[i][col - 1] = (M[i][col - 1] + M[i - 1][col - 1] + M[i - 1][col - 2] + M[i][col - 2] + M[i + 1][col - 2] + M[i + 1][col - 1]) / 6;
                }
                else{
                    res[i][j] = (M[i][j] + M[i][j - 1] + M[i - 1][j - 1] + M[i - 1][j] + M[i - 1][j + 1] + M[i][j + 1] + M[i + 1][j + 1] + M[i + 1][j] + M[i + 1][j - 1]) / 9;
                }
            }
        return res;
    }
};
