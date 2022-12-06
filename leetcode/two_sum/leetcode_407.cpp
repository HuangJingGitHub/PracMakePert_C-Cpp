// Extension of the 1D case to the 2D case with additional DFS will not work here.
class Solution {
public:
    int trapRainWater(vector<vector<int>>& heightMap) {
        int m = heightMap.size(), n = heightMap[0].size(), res = 0;
        vector<vector<int>> rowLeftMax(m, vector<int>(n, 0)), rowRightMax = rowLeftMax,
                            colUpMax = rowLeftMax, colDownMax = rowLeftMax, trapMax = rowLeftMax;
        vector<vector<bool>> visited(m, vector<bool>(n, false));

        for (int i = 0; i < m; i++) {
            rowLeftMax[i][0] = heightMap[i][0];
            rowRightMax[i].back() = heightMap[i].back();
        }
        for (int i = 0; i < n; i++) {
            colUpMax[0][i] = heightMap[0][i];
            colDownMax.back()[i] = heightMap.back()[i];
        }

        for (int row = 1; row <= m - 2; row++) {
            for (int col = 1; col <= n - 1; col++)
                rowLeftMax[row][col] = max(rowLeftMax[row][col - 1], heightMap[row][col - 1]);
            for (int col = n - 2; col >= 0; col--)
                rowRightMax[row][col] = max(rowRightMax[row][col + 1], heightMap[row][col + 1]);
        }
        for (int col = 1; col <= n - 2; col++) {
            for (int row = 1; row <= m - 1; row++)
                colUpMax[row][col] = max(colUpMax[row - 1][col], heightMap[row - 1][col]);
            for (int row = m - 2; row >= 0; row--)
                colDownMax[row][col] = max(colDownMax[row + 1][col], heightMap[row + 1][col]);
        }
        
        int rowHeight, colHeight, boundaryHeight;
        for (int row = 1; row <= m - 2; row++)
            for (int col = 1; col <= n - 2; col++) {
                rowHeight = min(rowLeftMax[row][col], rowRightMax[row][col]);
                colHeight = min(colUpMax[row][col], colDownMax[row][col]);
                boundaryHeight = min(rowHeight, colHeight);
                if (boundaryHeight > heightMap[row][col])
                    trapMax[row][col] = boundaryHeight;
            }

        for (auto& row : trapMax) {
            for (int& water : row)
                cout << water << " ";
            cout << '\n';
        }

        for (int row = 1; row <= m - 2; row++)
            for (int col = 1; col <= n - 2; col++) 
                if (trapMax[row][col] != 0) {
                    visited[row][col] = false;
                    dfs(trapMax, visited, row, col);
                }

        for (auto& row : trapMax) {
            for (int& water : row)
                cout << water << " ";
            cout << '\n';
        }

        for (int row = 1; row <= m - 2; row++)
            for (int col = 1; col <= n - 2; col++) 
                if (trapMax[row][col] != 0) {
                    int rowMinHeight = min(heightMap[row][col - 1], heightMap[row][col + 1]);
                    int colMinHeight = min(heightMap[row - 1][col], heightMap[row + 1][col]);
                    trapMax[row][col] = min(trapMax[row][col], min(rowMinHeight, colMinHeight));
                    res += trapMax[row][col] - heightMap[row][col];               
                }
        return res;
    }

    int dfs(vector<vector<int>>& trapMax, vector<vector<bool>>& visited, int row, int col) {
        if (trapMax[row][col] == 0)
            return 1e6;
        
        if (visited[row][col] == true)
            return trapMax[row][col];
        
        visited[row][col] = true;
        int rowMin = min(dfs(trapMax, visited, row, col - 1), dfs(trapMax, visited, row, col + 1));
        int colMin = min(dfs(trapMax, visited, row - 1, col), dfs(trapMax, visited, row + 1, col));
        trapMax[row][col] = min(trapMax[row][col], min(rowMin, colMin));
        return trapMax[row][col];
    }
};



// BFS
typedef pair<int, int> waterIdxPair;

class Solution {
public:
    int trapRainWater(vector<vector<int>>& heightMap) {
        int m = heightMap.size(), n = heightMap[0].size(), res = 0;
        if (m <= 2 || n <= 2)
            return 0;

        vector<vector<bool>> visited(m, vector<bool>(n, false));
        priority_queue<waterIdxPair, vector<waterIdxPair>, greater<waterIdxPair>> waterQueue;
        for (int col = 0; col < n; col++) {
            waterQueue.push(make_pair(heightMap[0][col], col));
            waterQueue.push(make_pair(heightMap.back()[col], n * (m - 1) + col));
            visited[0][col] = visited.back()[col] = true;
        }
        for (int row = 1; row <= m - 2; row++) {
            waterQueue.push(make_pair(heightMap[row][0], n * row));
            waterQueue.push(make_pair(heightMap[row].back(), n * (row + 1) - 1));
            visited[row][0] = visited[row].back() = true;
        }

        int direction_x[] = {-1, 1, 0, 0},
            direction_y[] = {0, 0, -1, 1};
        while (waterQueue.empty() == false) {
            waterIdxPair cur_min = waterQueue.top();
            waterQueue.pop();

            for (int i = 0; i < 4; i++) {
                int new_x = cur_min.second / n + direction_x[i],
                    new_y = cur_min.second % n + direction_y[i];
                
                if (new_x >= 0 && new_x < m && new_y >= 0 && new_y < n && visited[new_x][new_y] == false) {
                    if (heightMap[new_x][new_y] < cur_min.first)
                        res += cur_min.first - heightMap[new_x][new_y];

                    visited[new_x][new_y] = true;
                    waterQueue.push(make_pair(max(heightMap[new_x][new_y], cur_min.first), n * new_x + new_y));
                }
            }
        }
        return res;
    }
};
