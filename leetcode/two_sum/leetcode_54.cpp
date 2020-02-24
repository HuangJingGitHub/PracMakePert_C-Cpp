class Solution {
public:
    vector<int> spiralOrder(vector<vector<int>>& matrix) {
        vector<int> res;
        if (matrix.size() == 0 || matrix[0].size() == 0)
            return res;
        int row = matrix.size(), col = matrix[0].size(), counter = 0, i = 0, j = 0;
        while (counter < row * col){
            res.push_back(matrix[i][j]);
            
        }

    }
};
