class Solution {
public:
    int visited[1 << 21];
    bool canIWin(int maxChoosableInteger, int desiredTotal) {
        if (maxChoosableInteger >= desiredTotal)
            return true;
        if (maxChoosableInteger * (maxChoosableInteger + 1) / 2 < desiredTotal)
            return false;
        return dfs(0, 0, maxChoosableInteger, desiredTotal);
    }

    bool dfs(int state, int sum, int maxChoosableInteger, int desiredTotal) {
        if (visited[state] == 1)
            return true;
        if (visited[state] == 2)
            return false;

        for (int x = 1; x <= maxChoosableInteger; x++) {
            if ((1 << x) & state)
                continue;
            if (sum + x >= desiredTotal) {
                visited[state] = 1;
                return true;
            }
            if (dfs((1 << x) | state, sum + x, maxChoosableInteger, desiredTotal) == false) {
                visited[state] = 1;
                return true;
            }
        }
        visited[state] = 2;
        return false;
    }
};
