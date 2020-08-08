class Solution {
public:
    int maxSideLength(vector<vector<int>>& mat, int threshold) {
        if (mat.empty())
            return 0;

        int m = mat.size(), n = mat[0].size();
        vector<vector<int>> integralMat(m+1, vector<int>(n+1, 0));   // use m+1 x n+1 to avoid manual initialization of first row and first column.
        for (int i = 1; i <= m; i++)
            for (int j = 1; j <= n; j++){
                integralMat[i][j] = mat[i-1][j-1] + integralMat[i-1][j] + integralMat[i][j-1] - integralMat[i-1][j-1];
            }

        int res = 0, maxLen = min(m, n);
        for (int k = 1; k <= maxLen; k++)
            for (int i = 1; i <= m; i++)
                for (int j = 1; j <= n; j++){
                    if (i - k < 0 || j - k < 0)
                        continue;
                    int temp = integralMat[i][j] - integralMat[i-k][j] - integralMat[i][j-k] + integralMat[i-k][j-k];
                    if (temp <= threshold)
                        res = max(res, k);
                }
        return res;
    }
};
