class Solution {
public:
    int maximalNetworkRank(int n, vector<vector<int>>& roads) {
        int res = 0;
        vector<int> degrees(n, 0);
        vector<vector<bool>> adj_table(n, vector<bool>(n, false));
        for (auto& road : roads) {
            degrees[road[0]]++;
            degrees[road[1]]++;
            adj_table[road[0]][road[1]] = true;
            adj_table[road[1]][road[0]] = true;
        }
        for (int i = 0; i < n; i++)
            for (int j = i + 1; j < n; j++) {
                if (adj_table[i][j] == true)
                    res = max(res, degrees[i] + degrees[j] - 1);
                else
                    res = max(res, degrees[i] + degrees[j]);
            }
        return res;
    }
};
