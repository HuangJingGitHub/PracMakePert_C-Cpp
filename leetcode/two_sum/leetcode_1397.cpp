class Solution {
public:
private:
    vector<int> fail;
    int f[500][50][4];
    int trans[50][26];
    static constexpr int mod = 1000000007;
    string s1, s2, evil;

public:
    int getTrans(int stats, char ch) {
        if (trans[stats][ch - 97] != -1)
            return trans[stats][ch - 97];

        while (stats && evil[stats] != ch)
            stats = fail[stats - 1];
        return trans[stats][ch - 97] = (evil[stats] == ch ? stats + 1 : 0);
    }

    int dfs(int pos, int stats, int bound) {
        if (stats == evil.size())
            return 0;
        
        if (pos == s1.size())
            return 1;
        
        if (f[pos][stats][bound] != -1) 
            return f[pos][stats][bound];
        
        f[pos][stats][bound] = 0;
        char l = (bound & 1 ? s1[pos] : 'a');
        char r = (bound & 2 ? s2[pos] : 'z');  
        for (char ch = l; ch <= r; ch++) {
            int next_stats = getTrans(stats, ch);
            int next_bound = (bound & 1 ? ch == s1[pos] : 0) ^ (bound & 2 ? (ch == s2[pos]) << 1 : 0);
            f[pos][stats][bound] += dfs(pos + 1, next_stats, next_bound);
            f[pos][stats][bound] %= mod;
        }
        return f[pos][stats][bound];
    }

    int findGoodStrings(int n, string _s1, string _s2, string _evil) {
        s1 = _s1;
        s2 = _s2;
        evil = _evil;

        int m = evil.size();
        fail.resize(m);
        for (int i = 1; i < m; i++) {
            int j = fail[i - 1];
            while (j && evil[j] != evil[i])
                j = fail[j - 1];
            if (evil[j] == evil[i])
                fail[i] = j + 1;
        }
        memset(f, - 1, sizeof(f));
        memset(trans, -1, sizeof(trans));
        return dfs(0, 0, 3);
    }
};
