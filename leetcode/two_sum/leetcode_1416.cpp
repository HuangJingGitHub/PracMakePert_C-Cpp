// use dp, but pay attention to the number range in this problem.
class Solution {
public: 
    long long int MOD = 1000000007;

    int numberOfArrays(string s, int k) {
        vector<long long int> dp(s.size(), 0);
        if (stoi(s.substr(0, 1)) >= 1 && stoi(s.substr(0, 1)) <= k)
            dp[0] = 1;

        int digits = 0, k1 = k;
        while (k1 != 0){
            k1 /= 10;
            digits++;
        }

        long long int num;
        for (int i = 1; i < s.size(); i++){
            for (int j = max(1, i - digits + 1); j <= i; j++){
                if (s[j] == '0')
                    continue;
                num = stoll(s.substr(j, i - j + 1));
                if (num >= 1 && num <= k)
                    dp[i] += dp[j-1];
            }

            if (i < digits && s[0] != '0'){
                num = stoll(s.substr(0, i + 1));
                if (num >= 1 && num <= k)
                    dp[i]++;
            }
            dp[i] %= MOD;
        }

        return dp.back();
    }
};
