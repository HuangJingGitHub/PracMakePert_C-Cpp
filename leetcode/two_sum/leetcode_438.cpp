class Solution {   
public:
    vector<int> findAnagrams(string s, string p)
        vector<int> res;
        unordered_map<char, int> charFrequency;
        set<char> charSet;
        for (char ch : p) {
            charFrequency[ch]++;
            charSet.insert(ch);
        }

        vector<unordered_map<char, int>> occurenceLog(s.size());
        for (auto it = charSet.begin(); it != charSet.end(); it++)
            occurenceLog[0][*it] = 0;
        for (int i = 1; i < s.size(); i++)
            occurenceLog[i] = occurenceLog[i - 1];

        if (charSet.find(s[0]) != charSet.end())
            occurenceLog[0][s[0]]++;
        for (i = 1; i < s.size(); i++) {
            occurenceLog[i] = occurenceLog[i - 1];
            if (charSet.find(s[i]) != charSet.end())
                occurenceLog[i][s[i]]++;
        }

        for (int i = 0; i <= s.size() - p.size(); i++) 
            if (charSet.find(s[i]) != charSet.end()) {
                int endIdx = i + p.size() - 1;
                if (occurenceLog[endIdx][s[i]] - occurenceLog[i][s[i]]] + 1 != charFrequency[s[i]])
                    continue;

                auto it = charSet.begin();
                for (; it != charSet.end(); it++) {
                    char curChar = *it;
                    if (curChar == s[i])
                        continue;

                    if (occurenceLog[endIdx][curChar] - occurenceLog[i][curChar] != charFrequency[curChar])
                        break;
                }
                if (it == charSet.end())
                    res.push_back(i);
            }

            return res;
    }
};
