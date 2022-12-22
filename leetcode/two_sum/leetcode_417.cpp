class Solution {
public:
    vector<vector<int>> pacificAtlantic(vector<vector<int>>& heights) {
        int rowNum = heights.size(), colNum = heights[0].size();
        vector<vector<bool>> canFlowToPacific(rowNum, vector<bool>(colNum, false)),
                             canFlowToAltantic(rowNum, vector<bool>(colNum, false));
        vector<vector<int>> res;

        for (int row = 0; row < rowNum; row++) {
            dfs(heights, canFlowToPacific, row, 0);
            dfs(heights, canFlowToAltantic, row, colNum - 1);
        }
        for (int col = 0; col < colNum; col++) {
            dfs(heights, canFlowToPacific, 0, col);
            dfs(heights, canFlowToAltantic, rowNum - 1, col);
        }

        for (int row = 0; row < rowNum; row++)
            for (int col = 0; col < colNum; col++)
                if (canFlowToPacific[row][col] == true && canFlowToAltantic[row][col] == true)
                    res.push_back({row, col});
        
        return res;
    }

    void dfs(vector<vector<int>>& heights, vector<vector<bool>>& canFlow, int row, int col) {
        if (canFlow[row][col] == true)
            return;
        canFlow[row][col] = true;

        if (row - 1 >= 0 && heights[row - 1][col] >= heights[row][col])
            dfs(heights, canFlow, row - 1, col);
        if (row + 1 < heights.size() && heights[row + 1][col] >= heights[row][col])
            dfs(heights, canFlow, row + 1, col);  
        if (col - 1 >= 0 && heights[row][col - 1] >= heights[row][col])
            dfs(heights, canFlow, row, col - 1);  
        if (col + 1 < heights[0].size() && heights[row][col + 1] >= heights[row][col])
            dfs(heights, canFlow, row, col + 1);                                
    }
};
