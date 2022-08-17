class Solution {
public:
    int countPalindromicSubsequence(string s) {
        int res  = 0;
        map<int, vector<int>> charToindex;

        for (int i = 0; i < s.size(); i++) 
            charToindex[s[i]].push_back(i);
        
        for (auto it = charToindex.begin(); it != charToindex.end(); it++) {
            if (it->second.size() < 2)
                continue;
            int startIdx = it->second.front(), endIdx = it->second.back();
            set<char> distinctChar;
            for (int i = startIdx + 1; i < endIdx; i++)
                distinctChar.insert(s[i]);
            res += distinctChar.size();
        }

        return res;
    }
};
