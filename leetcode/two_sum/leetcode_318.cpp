class Solution {
public:
    int maxProduct(vector<string>& words) {
        int res = 0;
        vector<vector<pair<int, int>>> letterLog(26);
        for (int i = 0; i < words.size(); i++)
            for (char ch : words[i])
                letterLog[ch - 'a'].push_back(pair<int, int>(words[i].size(), i));
        
        for (auto& log : letterLog)
            sort(log.begin(), log.end());
        
        for (int i = 0; i < words.size(); i++) {
            unordered_set<int> curLog;
            for (char ch : words[i])
                curLog.push(ch - 'a');
            for(int )
        }
    }
};
