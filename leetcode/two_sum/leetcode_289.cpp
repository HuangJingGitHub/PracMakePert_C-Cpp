class Solution {
public:
    void gameOfLife(vector<vector<int>>& board) {
        vector<vector<int>> neighborLog(board.size(), vector<int>(board[0].size(), 0));
        int rowNum = board.size(), colNum = board[0].size();
       
        for (int row = 0; row < rowNum; row++)
            for (int col = 0; col < colNum; col++) {
                int neighborRow = row - 1, neighborCol = col - 1;
                if (neighborRow >= 0) {
                    neighborCol = (col - 1 >= 0) ? col - 1 : col;
                    for (; neighborCol < colNum && neighborCol <= col + 1; neighborCol++)
                        neighborLog[row][col] += board[neighborRow][neighborCol];
                }
                
                if (col != 0 && col != colNum - 1)
                    neighborLog[row][col] += (board[row][col - 1] + board[row][col + 1]);
                else if (col == 0 && col + 1 < colNum)
                    neighborLog[row][col] += board[row][col + 1];
                else if (col == colNum - 1 && col - 1 >= 0)
                    neighborLog[row][col] += board[row][col - 1];
                
                neighborRow = row + 1, neighborCol = col - 1;
                if (neighborRow < rowNum) {
                    neighborCol = (col - 1 >= 0) ? col - 1 : col;
                    for (; neighborCol < colNum && neighborCol <= col + 1; neighborCol++)
                        neighborLog[row][col] += board[neighborRow][neighborCol];
                }
            }

            for (int row = 0; row < rowNum; row++)
                for (int col = 0; col < colNum; col++) {
                    if (board[row][col] == 1) {
                        if (neighborLog[row][col] < 2)
                            board[row][col] = 0;
                        else if (neighborLog[row][col] > 3)
                            board[row][col] = 0;
                    }
                    else {
                        if (neighborLog[row][col] == 3)
                            board[row][col] = 1;
                    }
                }
    }
};
