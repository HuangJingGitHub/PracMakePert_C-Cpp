// The solution uses the idea of the offical solution, which is good. And some details in the C++ implementation is noteworthy.  
class Solution {
public:
    bool isValidSudoku(vector<vector<char>>& board) {
        vector<std::map<int, int>> rowLog(9), columnLog(9), subboxLog(9);   // The vector size must be declared first befor following indexing.

        int digit, boxIndex;
        for (int i = 0; i < 9; i++)
            for (int j = 0; j < 9; j++) {
                if (board[i][j] == '.')
                    continue;
                else {
                    digit =  board[i][j] - '0';     // Pay attention how to convert char number to int. Do not use std::stoi() as stoi() converts str not char.
                    boxIndex = i / 3 + 3 * (j / 3); 
                    rowLog[i][digit]++;
                    columnLog[j][digit]++;
                    subboxLog[boxIndex][digit]++;
                }

                if (rowLog[i][digit] > 1 || columnLog[j][digit] > 1 || 
                    subboxLog[boxIndex][digit] > 1)
                    return false;
            }
        return true;  
    }
};


// use set
class Solution {
public:
    bool isValidSudoku(vector<vector<char>>& board) {
        vector<set<int>> rowLog(9), columnLog(9), subboxLog(9);

        int digit, boxIndex;
        for (int row = 0; row < 9; row++)
            for (int col = 0; col < 9; col++) {
                if (board[row][col] == '.')
                    continue;
                else {
                    digit =  board[row][col] - '0';
                    boxIndex = col / 3 + 3 * (row / 3);
                    
                    if (rowLog[row].find(digit) != rowLog[row].end())
                        return false;  
                    rowLog[row].insert(digit);
                    if (columnLog[col].find(digit) != columnLog[col].end())
                        return false;
                    columnLog[col].insert(digit);
                    if (subboxLog[boxIndex].find(digit) != subboxLog[boxIndex].end())
                        return false;
                    subboxLog[boxIndex].insert(digit);
                }
            }
        return true;  
    }
};
