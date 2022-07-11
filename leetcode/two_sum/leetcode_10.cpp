class Solution {
public:
    bool isMatch(string s, string p) {
        if (p.size() == 0)
            return s.size() == 0;
        
        bool first = false;
        if (s.size() > 0 && (p[0] == s[0] || p[0] == '.'))
            first = true;
        
        if (p.size() >= 2 && p[1] == '*')
            return isMatch(s, p.substr(2)) || (first && isMatch(s.substr(1), p));
        else
            return first && isMatch(s.substr(1), p.substr(1));
    }
};


class Solution {
public:
    bool isMatch(string s, string p) {
        auto match = [&] (int i, int j) {
            if (i == 0)
                return false;
            if (p[j - 1] == '.')
                return true;
            else
                return s[i - 1] == p[j - 1];
        };

        vector<vector<int>> dp(s.size() + 1, vector<int>(p.size() + 1, false));
        dp[0][0] = true;

        for (int i = 0; i <= s.size(); i++) {
            for (int j = 1; j <= p.size(); j++) {
                if (p[j - 1] == '*') {
                    dp[i][j] |= dp[i][j - 2];
                    if (match(i, j - 1))
                        dp[i][j] |= dp[i - 1][j];
                }
                else {
                    if (match(i, j))
                        dp[i][j] |= dp[i - 1][j - 1];
                }
            }
        }
        return dp.back().back();
    }
};
