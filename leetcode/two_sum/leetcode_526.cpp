// backtrace
class Solution {
public:
    int countArrangement(int N) {
        int res = 0;
        vector<bool> visited(N, false);
        vector<vector<int>> feasiblePos(N);

        for (int i = 1; i <= N; i++)
            for (int j = 1; j <= N; j++)
                if (i % j == 0 || j % i == 0)
                    feasiblePos[i - 1].push_back(j);
                
        backTrace(1, N, res, feasiblePos, visited);
        return res;
    }

    void backTrace(int num, int N, int& res, vector<vector<int>>& feasiblePos, vector<bool>& visited) {
        if (num == N) {
            for (int posN : feasiblePos[N-1])  // Check if the last position is feasible for N.
                if (!visited[posN - 1]) 
                    res++;
            return;
        }

        for (int i = 0; i < feasiblePos[num - 1].size(); i++) {
            int curPos = feasiblePos[num - 1][i];
            if (visited[curPos - 1])
                continue;
            else {
                visited[curPos - 1] = true;
                backTrace(num + 1, N, res, feasiblePos, visited);
                visited[curPos - 1] = false;
            }
        }
    }
};
