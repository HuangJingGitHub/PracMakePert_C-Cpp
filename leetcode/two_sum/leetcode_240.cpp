class Solution {
public:
    bool searchMatrix(vector<vector<int>>& matrix, int target) {
        int m = matrix.size(), n = matrix[0].size(),
            row = 0, col = 0;
        while (row < m && col < n) {
            if (binarySearchRowCol(matrix, row, col, m, n, target) || binarySearchRowCol(matrix, row, col, m, n, target, false, true))
                return true;
            row++;
            col++;
        }
        return false;
    }
    
    bool binarySearchRowCol(vector<vector<int>>& matrix, int row, int col, int rows, int cols, int target, 
                         bool rowSearch = true, bool colSearch = false) {                           
        if (rowSearch) {
            int left = col, right = cols - 1, mid;
            while (left <= right) {
                mid = left + (right - left) / 2;
                if (matrix[row][mid] == target)
                    return true;
                if (matrix[row][mid] < target)
                    left = mid + 1;
                else
                    right = mid - 1;
            }
        }
        else if (colSearch) {
            int up = row, down = rows - 1, mid;
            while (up <= down) {
                mid = up + (down - up) / 2;
                if (matrix[mid][col] == target)
                    return true;
                if (matrix[mid][col] < target)
                    up = mid + 1;
                else    
                    down = mid - 1;
            }
        }
        return false;
    }
};
