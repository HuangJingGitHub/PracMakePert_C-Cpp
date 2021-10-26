class Solution {
public:
    int maximalSquare(vector<vector<char>>& matrix) {
        if (matrix.empty() || matrix[0].empty())
            return 0;

        int row = matrix.size(), col = matrix[0].size(), resLen  = 0;
        vector<vector<int>> dp(row, vector<int>(col, 0));

        for (int i = 0; i < col; i++)
            if (matrix[0][i] == '1') {
                dp[0][i] = 1;
                resLen = 1;
            }
        for (int i = 1; i < row; i++)
            if (matrix[i][0] == '1') {
                dp[i][0] = 1;
                resLen = 1;
            }
    
        for (int i = 1; i < row; i++)
            for (int j = 1; j < col; j++) {
                if (matrix[i][j] == '1') {
                    if (dp[i - 1][j - 1] == 0)
                        dp[i][j] = 1;
                    else {
                        int lastLen = dp[i - 1][j - 1], cnt = 1;
                        for ( ; cnt <= lastLen; cnt++) {
                            if (matrix[i][j - cnt] == '0' || matrix[i - cnt][j] == '0')
                                break;
                        }
                        dp[i][j] = cnt;    // or directly use the more efficient recurrence relation dp[i][j] = 1 + min({dp[i - 1][j - 1], dp[i][j - 1], dp[i - 1][j]}).
                    }
                    resLen = max(resLen, dp[i][j]);
                }
            }

        return resLen * resLen;
    }
};


class Solution {
public:
    int maximalSquare(vector<vector<char>>& matrix) {
        int rowNum = matrix.size(), colNum = matrix[0].size(), resLen = 0;
        vector<vector<int>> dp(rowNum, vector<int>(colNum, 0));

        for (int i = 0; i < colNum; i++)
            if (matrix[0][i] == '1') {
                dp[0][i] = 1;
                resLen = 1;
            }
        for (int i = 1; i < rowNum; i++)
            if (matrix[i][0] == '1') {
                dp[i][0] = 1;
                resLen = 1;
            }
        
        for (int row = 1; row < rowNum; row++)
            for (int col = 1; col < colNum; col++)
                if (matrix[row][col] == '1') {
                    dp[row][col] = min({dp[row - 1][col - 1], dp[row][col - 1], dp[row - 1][col]}) + 1;
                    resLen = max(resLen, dp[row][col]);
                }
        return resLen * resLen;
    }
};
