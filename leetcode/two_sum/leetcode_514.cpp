class Solution {
public:
    int findRotateSteps(string s, string t) {
        int n = s.length(), m = t.length();
        vector<vector<int>> vis(n, vector<int>(m + 1));
        vis[0][0] = true;
        vector<pair<int, int>> q = {{0, 0}};
        for (int step = 0;; step++) {
            vector<pair<int, int>> nxt;
            for (auto [i, j] : q) {
                if (j == m) {
                    return step;
                }
                if (s[i] == t[j]) {
                    if (!vis[i][j + 1]) {
                        vis[i][j + 1] = true;
                        nxt.emplace_back(i, j + 1);
                    }
                    continue;
                }
                for (int i2: {(i - 1 + n) % n, (i + 1) % n}) {
                    if (!vis[i2][j]) {
                        vis[i2][j] = true;
                        nxt.emplace_back(i2, j);
                    }
                }
            }
            q = move(nxt);
        }
    }
};
