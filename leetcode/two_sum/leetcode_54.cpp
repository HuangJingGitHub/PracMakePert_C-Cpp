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
