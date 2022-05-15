// word(string) passing by value: time 344 ms, storage: 151.3 MB
// word(string) passing by reference: time 24 ms, storage: 7.7 MB. Such a big difference!
class Solution {
public:
    bool exist(vector<vector<char>>& board, string word) {
        int m = board.size(), n = board[0].size();
        for (int row = 0; row < m; row++)
            for (int col = 0; col < n; col++) {
                if (board[row][col] == word[0]) 
                    if (dfs(0, board, word, row, col))
                        return true;
            }
        
        return false;
    }

    bool dfs(int wordIdx, vector<vector<char>>& board, string& word, int row, int col) {
        if (board[row][col] != word[wordIdx])
            return false;
        if (wordIdx == word.size() - 1)
            return true;
        
        char orginalCh = board[row][col];
        board[row][col] = '*';

        if ((row < board.size() - 1 && dfs(wordIdx + 1, board, word, row + 1, col))
            || (row > 0 && dfs(wordIdx + 1, board, word, row - 1, col))
            || (col > 0 && dfs(wordIdx + 1, board, word, row, col - 1))
            || (col < board[0].size() - 1 && dfs(wordIdx + 1, board, word, row, col + 1)))
            return true;
        
        board[row][col] = orginalCh;
        return false;
    }
};
