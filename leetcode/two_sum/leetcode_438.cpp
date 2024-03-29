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



class Solution {   
public:
    vector<int> findAnagrams(string s, string p) {
        if (s.size() < p.size())
            return {};

        vector<int> res;
        vector<int> charFrequency(26, 0), window(26, 0);
        for (char& ch : p)
            charFrequency[ch - 'a']++;
        
        int left = 0, right = 0, len = p.size();
        for (; right < len; right++)
            window[s[right] - 'a']++;
        
        right = len - 1;  // Remerber to return right to right - 1
        while (right < s.size()) {
            int i = 0;
            while (i < 26) {
                if (window[i] != charFrequency[i])
                    break;
                i++;
            }
            if (i == 26)
                res.push_back(left);
            
            window[s[left] - 'a']--;
            left++;
            right++;
            if (right == s.size())
                break;
            window[s[right] - 'a']++;
        }
        return res;
    }
};
