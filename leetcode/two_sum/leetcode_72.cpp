// DP is the right thought to solve the problem, quite stuble solution.
// Can refer to the discussion and solution sections on LeetCode.
class Solution {
public:
    int minDistance(string word1, string word2) {
        int m = word1.length(),
            n = word2.length();
        
        vector<vector<int>> cost(m+1, vector<int>(n+1));

        for (int i = 0; i < m+1; i++) 
            cost[i][0] = i;
        for (int i = 0; i < n+1; i++)
            cost[0][i] = i;

        for (int i = 1; i < m+1; i++)
            for (int j = 1; j < n+1; j++){
                if (word1[i-1] == word2[j-1])
                    cost[i][j] = cost[i-1][j-1];
                else
                    cost[i][j] = 1 + min(cost[i-1][j-1], min(cost[i][j-1], cost[i-1][j]));
            } 
        
        return cost[m][n];
    }
};
