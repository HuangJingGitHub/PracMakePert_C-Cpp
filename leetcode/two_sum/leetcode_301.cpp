class Solution {
public:
    bool isValid(string& str) {
        int cnt = 0;
        for (char& ch : str) {
            if (ch == '(')
                cnt++;
            else if (ch == ')')
                cnt--;
            
            if (cnt < 0)
                return false;
        }
        return cnt == 0;
    }

    vector<string> removeInvalidParentheses(string s) {
        unordered_set<string> level;
        level.insert(s);

        // BFS
        while (level.size()) {
            vector<string> levelRes;
            for (auto itr = level.begin(); itr != level.end(); itr++) {
                string curStr = *itr;
                if (isValid(curStr))
                    levelRes.push_back(curStr);
            }
            if (levelRes.size())
                return levelRes;
            
            unordered_set<string> nextLevel;
            for (auto itr = level.begin(); itr != level.end(); itr++) {
                string curStr = *itr;
                for (int i = 0; i < curStr.size(); i++) {
                    if (curStr[i] == '(' || curStr[i] == ')')
                        nextLevel.insert(curStr.substr(0, i) + curStr.substr(i+1));
                }
            }
            level = nextLevel;
        }

        return {};
    }
};
