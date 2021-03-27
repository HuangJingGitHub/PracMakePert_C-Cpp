// application of KMP algorithm
class Solution {
public:
    string longestPrefix(string s) {
        vector<int> next(s.size(), 0);
        next[0] = -1;
        int nextIdx = 1, strIdx = 0;
        
        while (nextIdx < s.size()) {
            if (s[nextIdx] == s[strIdx])
                next[nextIdx] = next[strIdx];
            else {
                next[nextIdx] = strIdx;
                while (strIdx >= 0 && s[nextIdx] != s[strIdx])
                    strIdx = next[strIdx];
            }
            nextIdx += 1;
            strIdx += 1;
        }
        return s.substr(0, strIdx);
    }
};
