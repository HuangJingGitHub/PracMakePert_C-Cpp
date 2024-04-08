class Solution {
public:
    vector<vector<int>> updateMatrix(vector<vector<int>>& mat) {
        queue<vector<int>> zero_pos_queue;
        int m = mat.size(), 
            n = mat[0].size();
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++) {
                if (mat[i][j] == 0)
                    zero_pos_queue.push({i, j});
                else
                    mat[i][j] = -1;
            }
        
        int dx[4] = {-1, 1, 0, 0},
            dy[4] = {0, 0, -1, 1};
        
        while (zero_pos_queue.empty() == false) {
            vector<int> cur_zero_pos = zero_pos_queue.front();
            zero_pos_queue.pop();
            int x = cur_zero_pos[0], 
                y = cur_zero_pos[1];
            for (int i = 0; i < 4; i++) {
                int new_x = x + dx[i],
                    new_y = y + dy[i];
                if (new_x >= 0 && new_x < m && new_y >= 0 && new_y < n
                    && mat[new_x][new_y] == -1) {
                    mat[new_x][new_y] = mat[x][y] + 1;
                    zero_pos_queue.push({new_x, new_y});
                }
            }
        }
        return mat;
    }
};
