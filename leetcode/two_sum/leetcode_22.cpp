// see the discussion https://leetcode-cn.com/problems/generate-parentheses/solution/zui-jian-dan-yi-dong-de-dong-tai-gui-hua-bu-lun-da/
class Solution {
public:
    vector<string> generateParenthesis(int n) {
        if (n == 0) return {};
        if (n == 1) return {"()"};

        vector<vector<string>> dp(n + 1);
        dp[0] = {""};
        dp[1] = {"()"};
        for (int i = 2; i <= n; i++) {
            for (int j = 0; j < i; j++) {
                for (string p : dp[j])
                    for (string q : dp[i - j - 1]) {
                        string str = "(" + p + ")" + q;
                        dp[i].push_back(str);    
                    }
            }
        }
        return dp[n];
    }
};


// dfs
class Solution {
public:
    vector<string> generateParenthesis(int n) {
        vector<string> res;
        dfs("", 0, 0, n, res);
        return res;
    }

    void dfs(string curStr, int left, int right, int n, vector<string>& res) {
        if (left == n && right == n) {
            res.push_back(curStr);
            return;
        }

        if (left < right)
            return;
        if (left < n)
            dfs(curStr + "(", left + 1, right, n, res);
        if (right < n)
            dfs(curStr + ")", left, right + 1, n, res);
    }
};
