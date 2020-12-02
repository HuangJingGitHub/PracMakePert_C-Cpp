// dp
class Solution {
public:
    bool wordBreak(string s, vector<string>& wordDict) {
        vector<bool> dpLog(s.size(), false);
        unordered_set<string> wordSet(wordDict.begin(), wordDict.end());

        for (int i = 0; i < s.size(); i++){
            if (wordSet.find(s.substr(0, i + 1)) != wordSet.end()){
                dpLog[i] = true;
                continue;
            }
            for (int j = 0; j < i; j++)
                if (dpLog[j] == true)
                    if (wordSet.find(s.substr(j + 1, i - j)) != wordSet.end()){
                        dpLog[i] = true;
                        break;
                    }
        }
        return dpLog.back();
    }
};
