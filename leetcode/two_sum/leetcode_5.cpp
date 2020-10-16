class Solution {
public:
    string longestPalindrome(string s) {
        if (s.size() <= 1)
            return s;
        
        int start = 0, len = 1;
        for (int i = 1; i < s.size(); i++){  // Similar to a dp idea. If the longest palindromic string contain s[i], its length can only be len + 1 or len + 2.
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
        if (endPos >= s.size() || len > endPos + 1)
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
