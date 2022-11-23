class Solution {
public:
    int nearestExit(vector<vector<char>>& maze, vector<int>& entrance) {
        int m = maze.size(), n = maze[0].size();
        
        vector<int> dx = {-1, 1, 0, 0},
                    dy = {0, 0, -1, 1};
        
        queue<tuple<int, int, int>> stepLog;
        stepLog.emplace(entrance[0], entrance[1], 0);
        maze[entrance[0]][entrance[1]] = '+';

        while (stepLog.empty() == false) {
            auto [cur_x, cur_y, cur_step] = stepLog.front();
            stepLog.pop();

            for (int i = 0; i < 4; i++) {
                int new_x = cur_x + dx[i],
                    new_y = cur_y + dy[i],
                    new_step = cur_step + 1;
                
                if (new_x >= 0 && new_x < m && new_y >= 0 && new_y < n && maze[new_x][new_y] == '.') {
                    if (new_x == 0 || new_x == m - 1 || new_y == 0 || new_y == n - 1)
                        return new_step;
    
                    maze[new_x][new_y] = '+';
                    stepLog.emplace(new_x, new_y, new_step);
                }
            }
        }
        return -1;
    }
};
