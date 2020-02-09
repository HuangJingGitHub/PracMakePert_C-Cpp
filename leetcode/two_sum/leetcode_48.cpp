// First transpose then get the mirror image.
class Solution {
public:
    void rotate(vector<vector<int>>& matrix) {
        int n = matrix.size(), temp;
        for(int i = 0; i < n; i++)
            for (int j = i; j < n; j++){  // pay attention to get transpose, j starts from i, not 0. Starting from 0 changes nothing
                temp = matrix[i][j];      // as it changes twice and recover the origional matrix.
                matrix[i][j] = matrix[j][i];
                matrix[j][i] = temp;
            }
        
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n/2; j++){
                temp = matrix[i][j];
                matrix[i][j] = matrix[i][n-1-j];
                matrix[i][n-1-j] = temp;
            }
    }
};
