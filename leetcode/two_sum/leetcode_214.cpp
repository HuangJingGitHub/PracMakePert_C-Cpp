class Solution {
public:
    string shortestPalindrome(string s) {
        string res;
        int len = s.size(), mid = 0;
        
        while (len < 2 * s.size()) {
            mid = len / 2 - (len - s.size());
            if (len % 2 == 0) {
                if (checkPalindrome(s, mid - 1, mid))
                    break;
            }
            else {
                if (checkPalindrome(s, mid, mid))
                    break;
            }
            len++;
        }

        res = s.substr(2 * s.size() - len);
        reverse(res.begin(), res.end());
        res += s;
        return res;
    }
    
    bool checkPalindrome(string& s, int left, int right) {
        while (left >= 0) {
            if (s[left] != s[right])
                return false;
            left--;
            right++;
        }
        return true;
    }
};
