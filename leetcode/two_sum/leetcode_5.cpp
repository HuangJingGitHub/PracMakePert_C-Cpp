class Solution {
public:
    string longestPalindrome(string s) {
        if (s.size() <= 1)
            return s;
        
        int start = 0, len = 1;
        for (int i = 1; i < s.size(); i++){  // Similar to a dp idea, if the longest palindromic string ends with s[i], starts with s[x] and its length is dp[i], 
                                             // its length can only be dp[i - 1] + 1 (s[x]-s[i - 1] is palindromic) or dp[i - 1] + 2（only s[x + 1]-s[i-1]
                                             // is palidromic）. 
            if (isPalindromic(s, i, len + 2)){
                len += 2;
                start = i - len + 1;
            }
            if (isPalindromic(s, i, len + 1)){
                len++;
                start = i - len + 1;
            }
        }

        return s.substr(start, len);
    }


    bool isPalindromic(const string& s, int endPos, int len){
        if (endPos >= s.size() || len > endPos + 1 || len < 1)
            return false;
        
        int left = endPos - len + 1, right = endPos;
        while (left < right){
            if (s[left] != s[right])
                return false;
            left++;
            right--;
        }

        return true;
    } 
};


// two dimensional dp solution
class Solution {
public:
    string longestPalindrome(string s) {
        vector<vector<bool>> dp(s.size(), vector<bool>(s.size(), false));
        int maxLen = 1, startIdx = 0;

        for (int i = s.size() - 1; i >= 0; i--)
            for (int j = i; j < s.size(); j++) {
                if (j - i <= 2) {
                    dp[i][j] = s[i] == s[j];
                }
                else {
                    dp[i][j] = (s[i] == s[j] && dp[i + 1][j - 1]);
                }
                if (dp[i][j] == true && j - i + 1 > maxLen) {
                    maxLen = j - i + 1;
                    startIdx = i;
                }
            }
        return s.substr(startIdx, maxLen);
    }
};
