// dp, just pay attention some details in this problem: start can be 1, there may be no invalid path reaching end.
class Solution {
public:
    vector<vector<int>> pathWithObstacles(vector<vector<int>>& obstacleGrid) {
        vector<vector<int>> res;
        if (obstacleGrid.empty() || obstacleGrid[0].empty() || obstacleGrid[0][0] == 1)
            return res;
        int row = obstacleGrid.size(), col = obstacleGrid[0].size();

        for (int i = 1; i < col; i++){
            if (obstacleGrid[0][i] == 1){
                for (int j = i; j < col; j++)
                    obstacleGrid[0][j] = INT_MAX;
                break;
            }
            else
                obstacleGrid[0][i] = i;
        }

        for (int i = 1; i < row; i++){
            if (obstacleGrid[i][0] == 1){
                for (int j = i; j < row; j++)
                    obstacleGrid[j][0] = INT_MAX;
                break;
            }
            else
                obstacleGrid[i][0] = i;
        }

        for (int i = 1; i < row; i++)
            for (int j = 1; j < col; j++){
                if (obstacleGrid[i][j] == 0 && !(obstacleGrid[i][j - 1] == INT_MAX && obstacleGrid[i - 1][j] == INT_MAX))
                    obstacleGrid[i][j] = min(obstacleGrid[i][j - 1], obstacleGrid[i - 1][j]) + 1;
                else
                    obstacleGrid[i][j] = INT_MAX;
            }

        if (obstacleGrid[row - 1][col - 1] == INT_MAX)
            return res;

        res = vector<vector<int>>(row + col - 1);
        for (int steps = row + col - 2, r = row - 1, c = col - 1; steps >= 0; steps--){
            res[steps] = {r, c};
            if (r > 0 && obstacleGrid[r - 1][c] == steps - 1){
                r--;
            }
            else
                c--;
        }
        return res;   
    }
};
