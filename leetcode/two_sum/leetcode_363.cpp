// O(m^2n^2)
class Solution {
public:
    int maxSumSubmatrix(vector<vector<int>>& matrix, int k) {
        int m = matrix.size(), n = matrix[0].size(), res = INT_MIN;
        vector<vector<int>> sumMatrix(m + 1, vector<int>(n + 1, 0));

        sumMatrix[1][1] = matrix[0][0];
        for (int col = 1; col <= n; col++)
            sumMatrix[1][col] = sumMatrix[1][col - 1] + matrix[1][col];
        for (int row = 1; row < m; row++)
            sumMatrix[row][1] = sumMatrix[row - 1][1] + matrix[row][1];
        
        for (int row = 2; row < m; row++)
            for (int col = 2; col < n; col++) {
                sumMatrix[row][col] = sumMatrix[row - 1][col] + sumMatrix[row][col - 1] - sumMatrix[row - 1][col - 1] + matrix[row][col];
            }
        
        for (int row = 1; row <= m; row++)
            for (int col = 1; col <= n; col++)
                for (int i = 1; i <= row; i++)
                    for (int j = 1; j <= col; j++) {
                        int curSum = sumMatrix[row][col] - sumMatrix[i - 1][col] - sumMatrix[row][j - 1] + sumMatrix[i - 1][j - 1]; 
                        if (curSum <= k)
                            res = max(curSum, res);
                    }
        return res;
    }
};
