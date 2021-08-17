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


class Solution {
public:
    int maxProduct(vector<string>& words) {
        map<int, int> bitHash;
        int bitMask = 0, res = 0;

        for (auto& str : words) {
            bitMask = 0;
            for (char ch : str)
                bitMask |= 1 << (ch - 'a');
            bitHash[bitMask] = max(bitHash[bitMask], (int)str.size());
        }

        for (auto it = bitHash.begin(); it != bitHash.end(); it++) {
            auto iter = ++it;
            it--;
            for (; iter != bitHash.end(); iter++)
                if ((it->first & iter->first) == 0)
                    res = max(res, it->second * iter->second);
        }
        return res;
    }
};
