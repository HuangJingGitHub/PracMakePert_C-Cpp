// will exceed the time limit
class Solution {
public:
    int maxProduct(vector<string>& words) {
        int res = 0;
        vector<set<int>> letterWordLog(26);
        vector<set<int>> commonLetterSet(words.size());
        vector<int> noCommonLetterMaxLen(words.size());
        
        for (int i = 0; i < words.size(); i++)
            for (char ch : words[i])
                letterWordLog[ch - 'a'].insert(i);

        for (int i = 0; i < words.size(); i++) {
            for (char ch : words[i]) {
                int letterIdx = ch - 'a';
                commonLetterSet[i].insert(letterWordLog[letterIdx].begin(), letterWordLog[letterIdx].end());
            }
        }

        for (int i = 0; i < words.size(); i++) {
            int maxLen = 0;
            for (int j = 0; j < words.size(); j++)
                if (commonLetterSet[i].find(j) == commonLetterSet[i].end() && words[j].size() > maxLen) {
                    noCommonLetterMaxLen[i] = words[j].size();
                    maxLen = words[j].size();
                }
        }

        for (int i = 0; i < words.size(); i++) 
            res = max(res, (int)words[i].size() * noCommonLetterMaxLen[i]);
        
        return res;
    }
};
