// brute force, two-layer loop, will exceed time limit
class Solution {
public:
    vector<string> findRepeatedDnaSequences(string s) {
        vector<string> res;
        unordered_set<string> visitedStr;
        if (s.size() <= 10)
            return res;

        for (int i = 0; i <= s.size() - 10; i++) {
            string curStr = s.substr(i, 10);
            if (visitedStr.find(curStr) != visitedStr.end()) 
                continue;
            
            visitedStr.insert(curStr);
            for (int j = i + 1; j <= s.size() - 10; j++) {
                int cnt = 0;
                while (cnt < 10) {
                    if (curStr[cnt] == s[j + cnt])
                        cnt++;
                    else
                        break;
                }
                if (cnt == 10) {
                    res.push_back(curStr);
                    break;
                }
            }
        }
        return res;
    }
};

// hash map, O(n) time and space complexity
class Solution {
public:
    vector<string> findRepeatedDnaSequences(string s) {
        if (s.size() <= 10)
            return {};
        
        vector<string> res;
        unordered_map<string, int> visitedStr;

        for (int i = 0; i <= (int)(s.size() - 10); i++) {
            string curStr = s.substr(i, 10);
            visitedStr[curStr]++;
            if (visitedStr[curStr] == 2) 
                res.push_back(curStr);
        }
        return res;
    }
};
