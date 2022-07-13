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


class Solution {
public:
    string longestCommonPrefix(vector<string>& strs) {
        int commonIdx = 0;
        for (; commonIdx < strs[0].size();) {
            int strIdx = 0;
            while (strIdx < strs.size() - 1 
                   && strs[strIdx][commonIdx] == strs[strIdx + 1][commonIdx])  // can choose not to check strs[strIdx + 1].size() > commonIdx as s[s.length] return null character
                strIdx++;
            if (strIdx == strs.size() - 1)
                commonIdx++;
            else
                break;
        }

        return strs[0].substr(0, commonIdx);
    }
};
