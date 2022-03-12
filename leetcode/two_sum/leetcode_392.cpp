class Solution {
public:
    bool isSubsequence(string s, string t) {
        if (s.size() > t.size())
            return false;
        
        int sIdx = 0, tIdx = 0;
        while (sIdx < s.size() && tIdx < t.size()) {
            if (s[sIdx] == t[tIdx]) {
                sIdx++;
                tIdx++;
            }
            else
                tIdx++;
        }

        return sIdx == s.size();
    }
};
