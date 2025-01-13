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


// A linear complexity algorithm, see the discussion panel for details.
class Solution {
public:
    int maximalNetworkRank(int n, vector<vector<int>>& roads) {
        vector<int> degrees(n, 0);
        vector<vector<bool>> adj_table(n, vector<bool>(n, false));
        for (auto& road : roads) {
            degrees[road[0]]++;
            degrees[road[1]]++;
            adj_table[road[0]][road[1]] = true;
            adj_table[road[1]][road[0]] = true;
        }
        int max_deg = degrees[0], max_idx = 0, second_deg = -1, second_idx;
        for (int degree : degrees) 
            max_deg = max(max_deg, degree);
        
        vector<int> max_idx_set;
        for (int i = 0; i < n; i++)
            if (degrees[i] == max_deg)
                max_idx_set.push_back(i);
        
        if (max_idx_set.size() == 1) {
            max_idx = max_idx_set[0];
            for (int i = 0; i < n; i++) {
                if (i == max_idx)
                    continue;
                if (degrees[i] > second_deg || (degrees[i] == second_deg && adj_table[max_idx][i] == false)) {
                    second_deg = degrees[i];
                    second_idx = i;
                }
            }
        }
        else {
            for (int i = 0; i < max_idx_set.size() - 1; i++) 
                for (int j = i + 1; j < max_idx_set.size(); j++) {
                    int max_idx_1 = max_idx_set[i], max_idx_2 = max_idx_set[j];
                    if (adj_table[max_idx_1][max_idx_2] == false)
                        return max_deg * 2;
                }
            return max_deg * 2 - 1;
        }

        if (adj_table[max_idx][second_idx] == true)
            return max_deg + second_deg - 1;
        return max_deg + second_deg;
    }
};
