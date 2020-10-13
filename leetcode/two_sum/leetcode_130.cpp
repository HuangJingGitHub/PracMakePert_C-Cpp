/** Similar to the island number problem, but it is not the same.
 * It is feasible to use the same technoque here, but what costs is
 * redundant comuptation as we have to mark the visited 'O' and recover it
 * after the DFS. There is not a good way to conquer this.
 * More effective way showing below is to find blocks starting with 
 * boundary 'O's and then continue processing.
 * Think about and analyse the problem first. Not how to apply existing experience to it first.
*/
class Solution {
public:
    void solve(vector<vector<char>>& board) {
        for (int row  = 0; row < board.size(); row++)
            for (int col = 0; col < board[0].size(); col++){
                bool isBorder = (row == 0 || row == board.size() - 1 || col == 0 || col == board[0].size() - 1);
                    if (isBorder && board[row][col] == 'O')
                        DFS_BorderProcess(board, row, col);
                }             
        
        for (int row = 0; row < board.size(); row++)
            for (int col = 0; col < board[0].size(); col++){
                if (board[row][col] == 'V')
                    board[row][col] = 'O';
                else if (board[row][col] == 'O')
                    board[row][col] = 'X';
            }
    }

    void DFS_BorderProcess(vector<vector<char>>& board, int row, int col){
        if (row < 0 || row >= board.size() || col < 0 || col >= board[0].size())
            return;
        if (board[row][col] != 'O')
            return;

        board[row][col] = 'V';
        DFS_BorderProcess(board, row, col - 1);
        DFS_BorderProcess(board, row, col + 1);
        DFS_BorderProcess(board, row - 1, col);
        DFS_BorderProcess(board, row + 1, col);
    }
};
