class Solution {
public:
    int longestPalindrome(string s) {
        int res = 0;
        unordered_map<char, int> charFrequency;

        for (int i = 0; i < s.size(); i++)
            charFrequency[s[i]]++;
        
        bool oddAdded = false;
        for (auto it = charFrequency.begin(); it != charFrequency.end(); it++) {
            if (it->second % 2 == 0)
                res += it->second;
            else if (oddAdded == false) {
                res += it->second;
                oddAdded = true;
            }
            else
                res += it->second - 1;

        }

        return res;
    }
};
