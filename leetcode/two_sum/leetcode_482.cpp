class Solution {
public:
    string licenseKeyFormatting(string s, int k) {
        string res;

        int cnt = 0;
        for (int i = s.size() - 1; i >= 0; i--) {
            if (s[i] == '-')
                continue;
            
            res += s[i];
            cnt++;
            if (cnt == k) {
                res += '-';
                cnt = 0;
            }
        }

        if (res.size() == 0)
            return res;

        if (res.back() == '-')
            res.pop_back();
        reverse(res.begin(), res.end());
        std::transform(res.begin(), res.end(), res.begin(), std::ptr_fun<int, int>(std::toupper));
        return res;
    }
};
