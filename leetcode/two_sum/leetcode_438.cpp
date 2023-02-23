class Solution {   
public:
    vector<int> findAnagrams(string s, string p) {
        if (s.size() < p.size())
            return {};
            
        vector<int> res;
        unordered_map<char, int> charFrequency;
        for (char ch : p) {
            charFrequency[ch]++;
        }

        vector<unordered_map<char, int>> occurenceLog(s.size());
        for (auto it = charFrequency.begin(); it != charFrequency.end(); it++)
            occurenceLog[0][it->first] = 0;
        for (int i = 1; i < s.size(); i++)
            occurenceLog[i] = occurenceLog[i - 1];

        if (charFrequency.find(s[0]) != charFrequency.end())
            occurenceLog[0][s[0]]++;
        for (int i = 1; i < s.size(); i++) {
            occurenceLog[i] = occurenceLog[i - 1];
            if (charFrequency.find(s[i]) != charFrequency.end())
                occurenceLog[i][s[i]]++;
        }

        for (int i = 0; i <= s.size() - p.size(); i++) 
            if (charFrequency.find(s[i]) != charFrequency.end()) {
                int endIdx = i + p.size() - 1;
                if (occurenceLog[endIdx][s[i]] - occurenceLog[i][s[i]] + 1 != charFrequency[s[i]])
                    continue;

                auto it = charFrequency.begin();
                for (; it != charFrequency.end(); it++) {
                    char curChar = it->first;
                    if (curChar == s[i])
                        continue;

                    if (occurenceLog[endIdx][curChar] - occurenceLog[i][curChar] != charFrequency[curChar])
                        break;
                }
                if (it == charFrequency.end())
                    res.push_back(i);
            }
            return res;
    }
};
