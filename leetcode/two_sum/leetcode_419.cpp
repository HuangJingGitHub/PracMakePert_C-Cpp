class Solution {
public:
    int countBattleships(vector<vector<char>>& board) {
        int res = 0;
        int m = board.size(), n = board[0].size();

        for (int row = 0; row < m; row++)
            for (int col = 0; col < n; col++) {
                if (board[row][col] == 'X') {
                    res++;
                    dfs(board, row, col);
                }
            }
        return res;
    }

    void dfs(vector<vector<char>>& board, int row, int col) {
        if (row < 0 || row >= board.size() || col < 0 || col >= board[0].size())
            return;
        if (board[row][col] == '.')
            return;
        
        board[row][col] = '.';
        dfs(board, row - 1, col);
        dfs(board, row + 1, col);
        dfs(board, row, col - 1);
        dfs(board, row, col + 1);
    }
};
