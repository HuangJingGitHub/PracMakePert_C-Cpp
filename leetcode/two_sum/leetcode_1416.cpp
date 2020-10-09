class Solution {
public:
    int numberOfArrays(string s, int k) {
        vector<int> dp(s.size(), 0);
        if (stoi(s.substr(0, 1)) >= 1 && stoi(s.substr(0, 1)) <= k)
            dp[0] = 1;

        int digits = 0, k1 = k;
        while (k1 != 0){
            k1 /= 10;
            digits++;
        }
        cout << digits << ' ';

        int num;
        for (int i = 1; i < s.size(); i++){
            for (int j = i - digits + 1; j >=1 && j <= i; j++){
                if (s[j] == '0')
                    continue;
                num = stoi(s.substr(j, i - j + 1));
                if (num >= 1 && num <= k)
                    dp[i] += dp[j-1];
            }

            if (i < digits && s[0] != '0'){
                num = stoi(s.substr(0, i + 1));
                if (num >= 1 && num <= k)
                    dp[i]++;
            }
        }

        return dp.back();
    }
};
