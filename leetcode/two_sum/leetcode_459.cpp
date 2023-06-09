class Solution {
public:
    vector<int> computeLPSArray(const string& pattern) {
        vector<int> lps(pattern.size(), 0);  // longest proper prefix also as a suffix in pattern[0...i]
        int len = 0, i = 1;
        while (i < pattern.size()) {
            if (pattern[i] == pattern[len]) {
                len++;
                lps[i] = len;
                i++;
            }
            else {
                if (len != 0)
                    len = lps[len - 1];
                else 
                    i++;
            }
        }
        return lps;
    }

    bool KMPmatch(const string& s, const string& pattern) {
        vector<int> lps = computeLPSArray(pattern);
        
        int idx_s = 0, idx_p = 0;
        while (s.size() - idx_s >= pattern.size() - idx_p) {
            if (s[idx_s] == pattern[idx_p]) {
                idx_s++;
                idx_p++;
            }

            if (idx_p == pattern.size())
                return true;
            else if (s[idx_s] != pattern[idx_p]) {
                if (idx_p != 0)
                    idx_p = lps[idx_p - 1];
                else
                    idx_s++;
            }
        }
        return false;
    }

    bool repeatedSubstringPattern(string s) {
        string double_s = s + s;
        double_s[0] = '*';
        double_s.back() = '*';
        return KMPmatch(double_s, s);
    }
};
