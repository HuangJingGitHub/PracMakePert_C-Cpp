class Solution {
public:
    int characterReplacement(string s, int k) {
        unordered_map<char, vector<int>> idxLog;
        for (int i = 0; i < s.size(); i++)
            idxLog[s[i]].push_back(i);
        
        return 0;
    }
};
