// dp
class Solution {
public:
    int maxValue(vector<vector<int>>& grid) {
        vector<vector<int>> giftDp(grid);
        int row = grid.size(), col = grid[0].size();
        for (int i = 1; i < col; i++)
            giftDp[0][i] += giftDp[0][i - 1];
        for (int i = 1; i < row; i++)
            giftDp[i][0] += giftDp[i - 1][0];
        
        for (int i = 1; i < row; i++)
            for (int j = 1; j < col; j++)
                giftDp[i][j] += max(giftDp[i - 1][j], giftDp[i][j - 1]);
        
        return giftDp.back().back();

    }
};
