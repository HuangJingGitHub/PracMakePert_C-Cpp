class Solution {
public:
    vector<int> dx{-1, 1, 0, 0, -1, 1, -1, 1};
    vector<int> dy{0, 0, -1, 1, -1, 1, 1, -1};

    vector<vector<char>> updateBoard(vector<vector<char>>& board, vector<int>& click) {
        int x = click[0], y = click[1];
        if (board[x][y] == 'M')
            board[x][y] = 'X';
        else
            dfs(board, x, y);
        return board;
    }

    void dfs(vector<vector<char>>& board, int x, int y) {
        int cnt = 0;
        for (int i = 0; i < 8; i++) {
            int next_x = x + dx[i],
                next_y = y + dy[i];
            
            if (next_x < 0 || next_x >= board.size() || next_y < 0 || next_y >= board[0].size())
                continue;
            if (board[next_x][next_y] == 'M')
                cnt++;
        }

        if (cnt > 0) {
            board[x][y] = cnt + '0';
            return;
        }

        board[x][y] = 'B';
        for (int i = 0; i < 8; i++) {
            int next_x = x + dx[i],
                next_y = y + dy[i];
            
            if (next_x < 0 || next_x >= board.size() || next_y < 0 || next_y >= board[0].size() || board[next_x][next_y] != 'E')
                continue;
            dfs(board, next_x, next_y);            
        }
    }
    
};
