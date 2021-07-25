class NumMatrix {
private:
    vector<vector<int>> cumSum; // matrix cumulative sum
    int rowNum;
    int colNum;
public:
    NumMatrix(vector<vector<int>>& matrix) {
        rowNum = matrix.size();
        colNum = matrix[0].size();
        cumSum = vector<vector<int>>(rowNum, vector<int>(colNum, 0));
        cumSum[0][0] = matrix[0][0];
        
        for (int col = 1; col < colNum; col++)
            cumSum[0][col] = cumSum[0][col - 1] + matrix[0][col];
        for (int row = 1; row < rowNum; row++)
            cumSum[row][0] = cumSum[row - 1][0] + matrix[row][0];
        for (int row = 1; row < rowNum; row++)
            for (int col = 1; col < colNum; col++)
                cumSum[row][col] = cumSum[row - 1][col] + cumSum[row][col - 1] 
                                    - cumSum[row - 1][col - 1] + matrix[row][col]; 
    }
    
    int sumRegion(int row1, int col1, int row2, int col2) {
        if (row1 == 0 && col1 == 0)
            return cumSum[row2][col2];
        else if (row1 == 0)
            return cumSum[row2][col2] - cumSum[row2][col1 - 1];
        else if (col1 == 0)
            return cumSum[row2][col2] - cumSum[row1 - 1][col2];
        else
            return cumSum[row2][col2] - cumSum[row2][col1 - 1] - cumSum[row1 - 1][col2]
                    + cumSum[row1 - 1][col1 - 1];
    }
};

/**
 * Your NumMatrix object will be instantiated and called as such:
 * NumMatrix* obj = new NumMatrix(matrix);
 * int param_1 = obj->sumRegion(row1,col1,row2,col2);
 */
