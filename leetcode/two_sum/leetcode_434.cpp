class Solution {
public:
    int countSegments(string s) {
        if (s.size() == 0)
            return 0;
        
        int res = 0, idx = 0;
        while (idx < s.size()) {
            if (s[idx] != ' ')
                res++;
            while (idx < s.size() && s[idx] != ' ')
                idx++;
            idx++;
        }

        return res;
    }
};
