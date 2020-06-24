// A very good example of traceback. Also pay attention to the details.
// Quite interesting is the speed between passing by value and by reference:
// word(string) passing by value: time 344 ms, storage: 151.3 MB
// word(string) passing by reference: time 24 ms, storage: 7.7 MB. Such a big difference!

class Solution {
public:
    bool traceback(int startRow, int startCol, vector<vector<char>>& board, string& word, int wordIdx)
    {
        if (board[startRow][startCol] != word[wordIdx]){
            return false;
        }

        if (wordIdx == word.size()-1){
            return true; 
        }

        char currentChar = board[startRow][startCol];
        board[startRow][startCol] = 0;
        wordIdx++;
        if ((startCol < board[0].size()-1 && traceback(startRow, startCol+1, board, word, wordIdx)) ||
            (startRow > 0 && traceback(startRow-1, startCol, board, word, wordIdx)) ||
            (startCol > 0 && traceback(startRow, startCol-1, board, word, wordIdx)) || 
            (startRow < board.size()-1 && traceback(startRow+1, startCol, board, word, wordIdx))){
                return true;
            }
        
        board[startRow][startCol] = currentChar;
        return false;

    }

    bool exist(vector<vector<char>>& board, string word) {
        if (board.empty() || board[0].empty())
            return false;
        
        int row = board.size(), col = board[0].size();

        for (int i = 0; i < row; i++)
            for (int j = 0; j < col; j++){
                    if (traceback(i, j, board, word, 0))
                        return true;
                }
        
        return false;
    }
};
