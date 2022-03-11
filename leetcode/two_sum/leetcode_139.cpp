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


// concise
class Solution {
public:
    bool wordBreak(string s, vector<string>& wordDict) {
        vector<bool> dp(s.size() + 1, false);
        dp.front() = true;
        unordered_set<string> wordSet(wordDict.begin(), wordDict.end());

        for (int i = 0; i < s.size(); i++) {
            for (int j = i; j >= 0; j--) {
                if (dp[j] == true && wordSet.find(s.substr(j, i - j + 1)) != wordSet.end()) {
                    dp[i + 1] = true;
                    break;
                }
            }
        }

        return dp.back();
    }
};
