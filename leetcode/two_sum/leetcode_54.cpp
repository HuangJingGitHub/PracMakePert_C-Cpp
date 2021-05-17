class Solution {
public:
    vector<int> spiralOrder(vector<vector<int>>& matrix) {
        if (matrix.size() == 0 || matrix[0].size() == 0){
            vector<int> res1;
            return res1;
        }
            
        int row = matrix.size(), col = matrix[0].size(), counter = 0, startRow = 0, startCol = 0, i, j;
        int length = row * col;
        vector<int> res(length);
        while (counter < length){
            for(i = startRow, j = startCol; j < col - startCol; j++){
                // res.push_back(matrix[i][j]);
                // counter++;
                res[counter++] = matrix[i][j];
            }
            if (counter == length)
                break;
            for(i = startRow + 1, j = col - startCol - 1; i < row - startRow; i++){
                res[counter++] = matrix[i][j];
            }
            if (counter == length)
                break;
            for(i = row - startRow - 1, j = col - startCol - 2; j >= startCol; j--){
                res[counter++] = matrix[i][j];
            }  
            if (counter == length)
                break;            
            for(i = row - startRow - 2, j = startCol; i > startCol; i--){
                res[counter++] = matrix[i][j];
            } 
            if (counter == length)
                break;            
            startRow++;
            startCol++;           
        }
        return res;
    }
};


// more compact and clear
class Solution {
public:
    vector<int> spiralOrder(vector<vector<int>>& matrix) {
        vector<int> res;
        int rows = matrix.size(), cols = matrix[0].size(), 
            startRow = 0, startCol = 0, 
            endRow = rows - 1, endCol = cols - 1;
        while (startRow <= endRow && startCol <= endCol) {
            spiralLevelTraversal(startRow, startCol, endRow, endCol, matrix, res);
            startRow++;
            startCol++;
            endRow--;
            endCol--;
        }
        return res;
    }

    void spiralLevelTraversal(int startRow, int startCol, int endRow, int endCol, vector<vector<int>>& matrix, vector<int>& res) {
        for (int i = startCol; i <= endCol; i++)
            res.push_back(matrix[startRow][i]);
        if (startRow == endRow)
            return;
        for (int i = startRow + 1; i <= endRow; i++)
            res.push_back(matrix[i][endCol]);
        if (startCol == endCol)
            return;
            
        for (int i = endCol - 1; i >= startCol; i--)
            res.push_back(matrix[endRow][i]);
        for (int i = endRow - 1; i > startRow; i--)
            res.push_back(matrix[i][startCol]);
    }
};
