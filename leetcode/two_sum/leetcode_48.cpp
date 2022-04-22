// Effect: i-th row becomes n - i th col. First transpose then get the mirror image. Anew[i][j] = Aold[n - 1 - j][i]
class Solution {
public:
    void rotate(vector<vector<int>>& matrix) {
        int n = matrix.size(), temp;
        for(int i = 0; i < n; i++)
            for (int j = i; j < n; j++) {  // pay attention to get transpose, j starts from i, not 0. Starting from 0 changes nothing
                temp = matrix[i][j];      // as it changes twice and recover the origional matrix.
                matrix[i][j] = matrix[j][i];
                matrix[j][i] = temp;
            }
        
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n/2; j++) {
                temp = matrix[i][j];
                matrix[i][j] = matrix[i][n-1-j];
                matrix[i][n-1-j] = temp;
            }
    }
};


class Solution {
public:
    void rotate(vector<vector<int>>& matrix) {
        int n = matrix.size();
        for (int i = 0; i < n; i++)
            for (int j = i; j < n; j++)
                swap(matrix[i][j], matrix[j][i]);
        
        for (int col = 0; col < n / 2; col++)
            for (int row = 0; row < n; row++)
                swap(matrix[row][col], matrix[row][n - 1 - col]);
    }
};
