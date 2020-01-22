class Solution {
public:
        static const int n = 3;
        static const int N = n * n;
        int rows[N][N+1];
        int columns[N][N+1];
        int boxes[N][N+1];
        bool sudokuSolved = false;
        vector<vector<char>> board;

    bool couldPlace(int d, int row, int col){
        int index = row / n + n * (col / n);
        return rows[row][d] + columns[col][d] + boxes[index][d] == 0;
    }

    void placeNumber(int d, int row, int col){
        int index = row / n + n * (col / n);
        rows[row][d]++;
        columns[col][d]++;
        boxes[index][d]++;
        board[row][col] = (char)(d + '0');
    }

    void removeNumber(int d, int row, int col){
        int index = row / n + n * (col / n);
        rows[row][d]--;
        columns[col][d]--;
        boxes[index][d]--;
        board[row][col] = '.';        
    }

    void placeNextNumber(int row, int col){
        if (row == N-1 && col == N-1)
            sudokuSolved = true;
        else{
            if (col == N-1) 
                backtrack(row+1, 0);
            else
                backtrack(row, col+1);
        }
    }

    void backtrack(int row, int col){
        if (board[row][col] == '.'){
            for (int d = 1; d < 10; d++){
                if (couldPlace(d, row, col)){
                    placeNumber(d, row, col);
                    placeNextNumber(row, col);
                    if (!sudokuSolved)
                        removeNumber(d, row, col);
                }
            }
        }
        else 
            placeNextNumber(row, col);
    }

    void solveSudoku(vector<vector<char>>& board){
        this->board = board;
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++){
                char num = board[i][j];
                if (num != '.'){
                    int d = num - '0';  
                    placeNumber(d, i, j);
                }
            }
        backtrack(0, 0);
    }      
};
