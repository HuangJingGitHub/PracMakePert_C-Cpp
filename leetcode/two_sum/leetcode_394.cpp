class Solution {
public:
    string decodeString(string s) {
        string res;
        stack<pair<int, string>> stk;

        int keyNum = 0;
        for (int i = 0; i < s.size(); i++) {
            if (s[i] == '[') {
                stk.push(pair<int, string>(keyNum, res));
                keyNum = 0;
                res = "";
            }            
            else if (s[i] == ']') {
                pair<int, string> lastPair = stk.top();
                stk.pop();
                string lastRes = lastPair.second;
                for (int j = 0; j < lastPair.first; j++)
                    lastRes += res;
                res = lastRes;
            }
            else if (s[i] >= '0' && s[i] <= '9')
                keyNum = keyNum * 10 + s[i] - '0';
            else
                res += s[i];
        }

        return res;
    }
};
