class Solution {
public:
    int projectionArea(vector<vector<int>>& grid) {
        if (grid.empty() || grid[0].empty())
            return 0;
        int res = 0, xyPlane = 0;
        vector<int> xzPlane(grid[0].size(), 0), yzPlane(grid.size(), 0);

        for (int row = 0; row < grid.size(); row++)
            for (int col = 0; col < grid[0].size(); col++){
                if (grid[row][col] > 0)
                    xyPlane++;
                xzPlane[col] = max(grid[row][col], xzPlane[col]);
                yzPlane[row] = max(grid[row][col], yzPlane[row]);
            }
        
        res = xyPlane;
        for (int num : xzPlane)
            res += num;
        for (int num : yzPlane)
            res += num;
        return res;
    }
};
