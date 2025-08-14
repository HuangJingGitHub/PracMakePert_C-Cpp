class Solution {
public:
    int shortestPathBinaryMatrix(vector<vector<int>>& grid) {
        vector<int> direction_x{-1, 0, 1}, direction_y{-1, 0, 1};
        if (grid[0][0] == 1)
            return -1;
        
        int n = grid.size(), res = 1;
        if (n == 1)
            return 1;
        
        vector<vector<bool>> visited(n, vector<bool>(n, false));
        queue<vector<int>> pos;
        pos.push({0, 0});
        visited[0][0] = true;

        while (!pos.empty()) {
            int level_size = pos.size();
            while (level_size > 0) {
                vector<int> cur_pos = pos.front();
                pos.pop();

                int x = cur_pos[0], y = cur_pos[1];
                if (x == n - 1 && y == n - 1) {
                    return res;
                }
                

                for (int x_dif : direction_x) {
                    for (int y_dif : direction_y) {
                        if (x_dif == 0 && y_dif == 0) {
                            continue;
                        }

                        int new_x = x + x_dif, new_y = y + y_dif;
                        if (new_x >= 0 && new_x < n
                            && new_y >= 0 && new_y < n
                            && grid[new_x][new_y] == 0
                            && !visited[new_x][new_y]) {
                            pos.push({new_x, new_y});
                            visited[new_x][new_y] = true;
                        }
                    }
                }
                level_size--;
            }
            res += 1;
        }
        return -1;
    }
};
