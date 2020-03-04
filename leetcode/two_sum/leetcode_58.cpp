class Solution {
public:
    int lengthOfLastWord(string s) {
        int n = s.size(), res = 0;
        for(int i = n - 1; i >= 0; i--){
            if (s[i] == ' ' && res == 0)
                continue;
            if (s[i] == ' ')
                return res;
            res++;
        }
        return res;
    }
};
