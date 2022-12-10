class Solution {
public:
    int longestPalindrome(string s) {
        int res = 0;
        unordered_map<char, int> charFrequency;

        for (int i = 0; i < s.size(); i++)
            charFrequency[s[i]]++;
        
        return res;
    }
};
