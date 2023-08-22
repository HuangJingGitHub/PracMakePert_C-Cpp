class Solution {
public:
    int findMaxForm(vector<string>& strs, int m, int n) {
        vector<vector<int>> dp(m + 1, vector<int>(n + 1, 0));

        for (int i = 1; i <= strs.size(); i++) {
            vector<int> zero_one_num = countZeroAndOne(strs[i - 1]);
            for (int j = m; j >= 0; j--)
                for (int k = n; k >=0; k--) {
                    int cur_zero_num = zero_one_num[0],
                        cur_one_num = zero_one_num[1];
                    if (j >= cur_zero_num && k >= cur_one_num) 
                        dp[j][k] = max(dp[j][k], dp[j - cur_zero_num][k - cur_one_num] + 1);
                }
        }
        return dp.back().back();
    }

    vector<int> countZeroAndOne(string& str) {
        int zero_num = 0;
        for (int i = 0; i < str.size(); i++)
            if (str[i] == '0')
                zero_num++;
        return {zero_num, (int)str.size() - zero_num};
    }
};
