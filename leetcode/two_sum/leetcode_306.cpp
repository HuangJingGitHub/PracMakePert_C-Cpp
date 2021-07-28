class Solution {
public:
    bool isAdditiveNumber(string num) {
        int i = 0;
        for (int j = i + 1; j <= num.size() - 1; j++)
            for (int k = j + 1; k <= num.size() - 1; k++)
                if (dfs(num, i, j, k))
                    return true;
        return false;
    }

    bool dfs(string& s, int i, int j, int k) {
        if ((s[i] == '0' && j - i > 1) || s[j] == '0' && k - j > 1)
            return false;
        string a = s.substr(i, j - i),
               b = s.substr(j, k - j),
               sum = add(a, b);
        int n = sum.size();
        if (k + n - 1 > s.size() - 1 || sum != s.substr(k, n))
            return false;
        if (k + n - 1 == s.size() - 1)
            return true;
        return dfs(s, j, k, k + n);
    }

    string add(string& a, string& b) {
        string res;
        int n1 = a.size() - 1,
            n2 = b.size() - 1;
        int carry = 0;
        
        while (n1 >= 0 || n2 >= 0 || carry > 0) {
            int t1 = n1 >= 0 ? a[n1--] - '0' : 0;
            int t2 = n2 >= 0 ? b[n2--] - '0' : 0;
            res += (t1 + t2 + carry) % 10 + '0';
            carry = (t1 + t2 + carry) >= 10 ? 1 : 0;
        }
        reverse(res.begin(), res.end());
        return res;
    }
};
