class Solution {
public:
    string longestCommonPrefix(vector<string>& strs) {
        if (strs.empty())
            return "";
        
        string res;
        int resLen = 0;
        while (true) {
            if (strs[0].size() == resLen)
                return res;
            char curChar = strs[0][resLen];
            for (int i = 1; i < strs.size(); i++) {
                if (strs[i].size() == resLen || strs[i][resLen] != curChar)
                    return res;
            }
            res += curChar;
            resLen++;
        }
        return res;
    }
};
