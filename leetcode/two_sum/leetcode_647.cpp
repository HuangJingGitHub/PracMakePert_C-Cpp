class Solution {
public:
    int countSubstrings(string s) {
        int res = 0;
        vector<vector<bool>> isPalindromic(s.size(), vector<bool>(s.size(), false));
        for (int startIdx = s.size() - 1; startIdx >= 0; startIdx--)
            for (int endIdx = startIdx; endIdx < s.size(); endIdx++) {
                if (endIdx - startIdx <= 2)
                    isPalindromic[startIdx][endIdx] = s[startIdx] == s[endIdx];
                else {
                    isPalindromic[startIdx][endIdx] = (isPalindromic[startIdx + 1][endIdx - 1] == true && s[startIdx] == s[endIdx]);
                }
                if (isPalindromic[startIdx][endIdx])
                    res++;
            }
        return res;
    }
};
