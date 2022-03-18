class Solution {
public:
    char findTheDifference(string s, string t) {
        vector<int> log(26, 0);
        
        for (char ch : s)
            log[ch - 'a']++;
        for (char ch : t)
            log[ch - 'a']--;
        
        int i = 0;
        for (; i < 26; i++)
            if (log[i])
                break;
        return 'a' + i;
    }
};
