class Solution {
public:
    typedef long long LL;
    static const int mod = 1e9 + 7;
    int f[1010][1010];
    int s[1010][1010];
    int kInversePairs(int n, int k) {
        memset(f, 0, sizeof f);
        memset(s, 0, sizeof s);
        f[0][0] = 1;
        
        for(int i = 0; i <= k; i++) {
            if (i == 0) 
                s[0][i] = f[0][i];
            else 
                s[0][i] = s[0][i - 1] + f[0][i];
        }
        for(int i = 1; i <= n; i++) {
            for(int j = 0; j <= k; j++) {
                if (j > i - 1) 
                    f[i][j] = (((LL)s[i - 1][j] - s[i - 1][j - i]) % mod + mod) % mod;
                else 
                    f[i][j] = s[i - 1][j];
                if (j == 0) 
                    s[i][j] = f[i][j];
                else 
                    s[i][j] = ((LL)s[i][j - 1] + f[i][j]) % mod; 
            }
        }
        return f[n][k];
    }
};
