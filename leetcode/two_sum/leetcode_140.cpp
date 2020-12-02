// backtrace
class Solution {
public:
    vector<string> wordBreak(string s, vector<string>& wordDict) {
        unordered_set<string> wordSet(wordDict.begin(), wordDict.end());
        vector<string> path, res;
        if (!wordBreakCheck(s, wordSet))
            return res;

        backTrace(s, 0, wordSet, path, res);
        return res;
    }

    void backTrace(string& s, int startIdx, unordered_set<string>& wordSet, vector<string>& path, vector<string>& res){
        if (startIdx == s.size()){
            string temp = path.front();
            for (int i = 1; i < path.size(); i++)
                temp += " " + path[i];
            res.push_back(temp);
            return;
        }

        for (int i = startIdx; i < s.size(); i++){
            if (wordSet.find(s.substr(startIdx, i - startIdx + 1)) == wordSet.end())
                continue;
            else{
                path.push_back(s.substr(startIdx, i - startIdx + 1));
                backTrace(s, i + 1, wordSet, path, res);
                path.pop_back();
            }
        }
    }

    bool wordBreakCheck(string& s, unordered_set<string>& wordSet) {
        vector<bool> dpLog(s.size(), false);

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
