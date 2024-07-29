class Solution {
public:
    int findIntegers(int n) {
        string s = bitset<32>(n).to_string();
        int m = s.size();
        int dp[m][2];
        memset(dp, -1, sizeof(dp));
        function<int(int, bool, bool)> f = [&](int i, bool is_pre_1, bool is_limit)->int {
            if (i == m) 
                return 1;
            if (is_limit == false && dp[i][is_pre_1] > 0)
                return dp[i][is_pre_1];
            
            int res = 0;
            for(int d = 0, up = is_limit ? s[i] - '0' : 1; d <= up; d++) {
                if (d == 1 && is_pre_1 == true)
                    continue;
                res += f(i + 1, d == 1, is_limit == true && d == up);
            }
            if (is_limit == false)
                dp[i][is_pre_1] = res;
            return res;
        };
        return f(0, false, true);
    }
};
