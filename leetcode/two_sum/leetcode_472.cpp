// dp
class Solution {
public:
    vector<string> findAllConcatenatedWordsInADict(vector<string>& words) {
        vector<string> res;
        if (words.size() < 2)
            return res;

        unordered_set<string> wordsDict;
        for (string str : words)
            wordsDict.insert(str);
        
        for (int i = 0; i < words.size(); i++){
            if (words[i].size() < 2)
                continue;

            string curStr = words[i];
            vector<bool> dp(curStr.size(), false);
            for (int j = 0; j < curStr.size() - 1; j++){      
                if (wordsDict.find(curStr.substr(0, j + 1)) != wordsDict.end()){
                    dp[j] = true;
                    continue;
                }
                else{
                    for (int k = 0; k < j; k++){
                        if (dp[k]){
                            if (wordsDict.find(curStr.substr(k + 1, j - k)) != wordsDict.end()){
                                dp[j] = true;
                                break;
                            }
                        }
                    }
                }
            }

            for (int i = 0; i < curStr.size() - 1; i++)
                if (dp[i])
                    if (wordsDict.find(curStr.substr(i + 1, curStr.size() - 1 - i)) != wordsDict.end()){
                        dp.back() = true;
                        break;
                    }
            
            if (dp.back())
                res.push_back(curStr);
        }

        return res;
    }
};
