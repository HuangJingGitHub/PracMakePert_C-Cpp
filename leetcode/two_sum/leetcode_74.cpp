class Solution {
public:
    bool searchMatrix(vector<vector<int>>& matrix, int target) {
        if (matrix.size() == 0 || matrix[0].size() == 0)
            return false;
        int row = matrix.size(), col = matrix[0].size(), searchRow = -1;
        // Find search row
        for (int i = 0; i < row; i++){
            if (!(matrix[i][0] <= target && matrix[i][col-1] >= target))
                continue;
            else{
                searchRow = i;
                break;
            }
        }

        if (searchRow == -1)
            return false;
        // Binary search
        for (int left = 0, right = col - 1, mid; left <= right; ){
            mid = (left + right) / 2;
            if (matrix[searchRow][mid] > target)
                right = mid - 1;
            else if (matrix[searchRow][mid] < target)
                left = mid + 1;
            else
                return true;
        }
        return false;
    }
};


√class Solution {
public:
    bool searchMatrix(vector<vector<int>>& matrix, int target) {
        int m = matrix.size(), n = matrix[0].size();
        int left = 0, right = m * n - 1, mid, midRow, midCol;

        while (left <= right) {
            mid = left + (right - left) / 2;
            midRow = mid / n;
            midCol = mid % n;
            if (matrix[midRow][midCol] == target)
                return true;
            else if (matrix[midRow][midCol] < target)
                left = mid + 1;
            else
                right = mid - 1;
        }
        return false;
    }
};
