class Solution {
public:
    int trapRainWater(vector<vector<int>>& heightMap) {
        int m = heightMap.size(), n = heightMap[0].size(), res = 0;
        vector<vector<int>> rowLeftMax(m, vector<int>(n, 0)), rowRightMax = rowLeftMax,
                            colUpMax = rowLeftMax, colDownMax = rowLeftMax;
        for (int i = 0; i < m; i++) {
            rowLeftMax[i][0] = heightMap[i][0];
            rowRightMax[i].back() = heightMap[i].back();
        }
        for (int i = 0; i < n; i++) {
            colUpMax[0][i] = heightMap[0][i];
            colDownMax.back()[i] = heightMap.back()[i];
        }

        for (int row = 1; row <= m - 2; row++) {
            for (int col = 1; col <= n - 2; col++)
                rowLeftMax[row][col] = max(rowLeftMax[row][col - 1], heightMap[row][col - 1]);
            for (int col = n - 2; n >= 0; col--)
                rowRightMax[row][col] = max(rowRightMax[row][col + 1], heightMap[row][col + 1]);
        }
        for (int col = 1; col <= n - 2; col++) {
            for (int row = 1; row <= m - 2; row++)
                colUpMax[row][col] = max(colUpMax[row - 1][col], heightMap[row - 1][col]);
            for (int row = m - 2; row >= 1; row--)
                colDownMax[row][col] = max(colDownMax[row + 1][col], heightMap[row + 1][col]);
        }
        
        int rowHeight, colHeight, boundaryHeight;
        for (int row = 1; row <= m - 2; row++)
            for (int col = 1; col <= n - 2; col++) {
                rowHeight = min(rowLeftMax[row][col], rowRightMax[row][col]);
                colHeight = min(colUpMax[row][col], colDownMax[row][col]);
                boundaryHeight = min(rowHeight, colHeight);
                if (boundaryHeight > heightMap[row][col]);
                    res += boundaryHeight - heightMap[row][col];
            }
        return res;
    }
};