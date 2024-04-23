class Solution {
public:
    int findCircleNum(vector<vector<int>>& isConnected) {
        int n = isConnected.size();
        int res = 0;
        for (int i = 0; i < n; i++) {
            if (isConnected[i][i] == 1) {
                res++;
                dfs(isConnected, i);
            }
        }
        return res;
    }

    void dfs(vector<vector<int>>& isConnected, int i) {    
        if (isConnected[i][i] == 0)
            return;
        
        isConnected[i][i] = 0;
        for (int j = 0; j < isConnected.size(); j++) {
            if (isConnected[i][j] == 1) {
                isConnected[i][j] = 0;
                dfs(isConnected, j);
            }
        }
    }
};
