class Solution {
public:
    int longestSubstring(string s, int k) {
        if (k <= 1)
            return s.size();
        if (s.empty() || s.size() < k)
            return 0;
        
        vector<int> hash(128, 0);
        for (char c : s)
            hash[c]++;
        
        int i = 0;
        while (i < s.size() && hash[s[i]] >= k) 
            i++;
        if (i == s.size())
            return s.size();
        
        int left = longestSubstring(s.substr(0, i), k);
        while (i < s.size() && hash[s[i]] < k)
            i++;
        int right = longestSubstring(s.substr(i), k);

        return max(left, right);
    }
};
