// 2D dp
class Solution {
public:
    bool isMatch(string s, string p) {
        if (s.empty()) {
            if (p.empty())
                return true;
            else
                for (char ch : p)
                    if (ch != '*')
                        return false;
                return true;
        }
        if (p.empty()) 
            return false;

        vector<vector<bool>> dp(s.size(), vector<bool>(p.size(), false));  // dp[i][j] represents result of 0-i substr of s and 0-j substr of p.
        // initialization
        if (s[0] == p[0] || p[0] == '?' || p[0] == '*')
            dp[0][0] = true;
        
        if (p[0] == '*')
            for (int i = 1; i < s.size(); i++)
                dp[i][0] = true;

        // It is a little subtle for the initialization of first row
        int matchedCnt = 0;
        if (p[0] == '?' || p[0] == s[0])
            matchedCnt = 1;
        for (int i = 1; i < p.size(); i++) {
            if (dp[0][i - 1]) {
                if (p[i] == '*')
                    dp[0][i] = true;
                else if (p[i] == '?' || p[i] == s[0]) {
                    if (matchedCnt == 0) {
                        dp[0][i] = true;
                        matchedCnt++;
                    }
                }
            }
        }

        for (int i = 1; i < s.size(); i++)
            for (int j = 1; j < p.size(); j++) {
                if (dp[i - 1][j - 1]) {
                    if (s[i] == p[j] || p[j] == '*' || p[j] == '?')
                        dp[i][j] = true;
                }
                if (dp[i - 1][j]) {
                    if (p[j] == '*')
                        dp[i][j] = true;
                }
                if (dp[i][j - 1]) {
                    if (p[j] == '*')
                        dp[i][j] = true;
                }
            }
        return dp.back().back();
    }
};
