class Solution {
public:
    struct data{
        int w, x, y;
        data(int w, int x, int y){
            this->w = w;
            this->x = x;
            this->y = y;
        }
        data(){

        }
    };
    int n, m;
    int visited[55][55];
    int dir_x[4] = {0, 0, -1, 1};
    int dir_y[4] = {1, -1, 0, 0};

    int bfs(vector<vector<int>>& forest, int start_x, int start_y, int target_x, int target_y){
        int step = 0;
        queue<pair<int, int>> q;
        q.push({start_x, start_y});
        visited[start_x][start_y] = 1;
        while(!q.empty()){
            int q_size = q.size();
            for(int i = 0; i < q_size; i++){
                int x = q.front().first;
                int y = q.front().second;
                q.pop();
                if(x == target_x && y == target_y) return step;
                for(int j = 0; j < 4; j++){
                    int xx = x + dir_x[j];
                    int yy = y + dir_y[j];
                    if(xx >= 0 && yy >= 0 && xx < m && yy < n && forest[xx][yy] && !visited[xx][yy]){
                        q.push({xx, yy});
                        visited[xx][yy] = 1;
                    }
                }
            }
            step++;
        }   
        return -1;
    }

    int cutOffTree(vector<vector<int>>& forest) {
        m = forest.size();
        n = forest[0].size();
        vector<data> arr;
        for(int i = 0; i < m; i++){
            for(int j = 0; j < n; j++){
                if(forest[i][j] > 1){
                    // 1是地面不是树
                    arr.push_back({forest[i][j], i, j});
                }
            }
        }
        auto cmp = [&](const data &a, const data &b){
            return a.w < b.w;//小到大
        };
        sort(arr.begin(), arr.end(), cmp);
        int pre_x = 0, pre_y = 0, ans = 0;
        for(auto [w, x, y] : arr){
            int step = bfs(forest, pre_x, pre_y, x, y);
            // cout << step << endl;
            if(step == -1) return -1;
            ans += step;
            memset(visited, 0, sizeof(visited));
            pre_x = x, pre_y = y;
        }
        return ans;
    }
};
