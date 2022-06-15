class Solution {
public:
    int firstUniqChar(string s) {
        int res = INT_MAX;
        vector<int> firstIdx(26, -1);

        for (int i = 0; s[i]; i++) {
            if (firstIdx[s[i] - 'a'] == -1)
                firstIdx[s[i] - 'a'] = i;
            else 
                firstIdx[s[i] - 'a'] = -2;
        }

        for (int idx : firstIdx) {
            if (idx >= 0)
                res = min(res, idx);
        }

        return res == INT_MAX ? -1 : res;
    }
};


class Solution {
public:
    int firstUniqChar(string s) {
        map<char, int> frequencyLog;
        
        for (char& ch : s)
            frequencyLog[ch]++;
        for (int i = 0; i < s.size(); i++)
            if (frequencyLog[s[i]] == 1)
                return i;
        return -1;
    }
};
