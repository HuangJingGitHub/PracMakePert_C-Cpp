class Solution {
public:
    vector<string> findWords(vector<vector<char>>& board, vector<string>& words) {
        if (board.empty() || board.front().empty())
            return {};

        vector<string> res;

        for (string curWord : words) {
            bool found = false;
            for (int row = 0; row < board.size(); row++) {
                for (int col = 0; col < board.front().size(); col++) {
                    if (backtrace(board, row, col, curWord, 0)) {
                        found = true;
                        res.push_back(curWord);
                        break;
                    }
                if (found)
                    bresak;
                }
            }
        }
        return res;
    }

    bool backtrace(vector<vector<char>>& board, int row, int col, string& word, int wordIdx) {
        if (board[row][col] != word[wordIdx])
            return false;
        if (wordIdx == word.size() - 1)
            return true;
        
        char originChar = board[row][col];
        board[row][col] = '*';
        if ((col < board.front().size() - 1 && backtrace(board, row, col + 1, word, wordIdx + 1)) ||
            (col > 0 && backtrace(board, row, col - 1, word, wordIdx + 1)) ||
            (row < board.size() - 1 && backtrace(board, row + 1, col, word, wordIdx + 1)) ||
            (row > 0 && backtrace(board, row - 1, col, word, wordIdx + 1))) {
                board[row][col] = originChar;
                return true;   
            }
        board[row][col] = originChar;
        return false;
    }
};
