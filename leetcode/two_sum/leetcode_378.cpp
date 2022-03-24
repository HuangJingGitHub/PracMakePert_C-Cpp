class Solution {
public:
    int kthSmallest(vector<vector<int>>& matrix, int k) {
        int n = matrix[0].size(), res;
        vector<int> idxInRow(n, 0);

        for (int i = 0; i < k; i++) {
            int moveRow = 0, tempMin = INT_MAX;
            for (int row = 0; row < n; row++) {
                if (idxInRow[row] < n) {
                    if (matrix[row][idxInRow[row]] < tempMin) {
                        moveRow = row;
                        tempMin = matrix[row][idxInRow[row]];
                    }
                }
            }
            idxInRow[moveRow]++;
            res = tempMin;
        }

        return res;
    }
};
