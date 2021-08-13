class Solution {
public:
    string removeDuplicateLetters(string s) {
        unordered_map<char, int> appearanceLog;
        unordered_map<char, bool> inStack;
        stack<char> charStack;

        for (char& ch : s)
            appearanceLog[ch]++;
        
        charStack.push(s[0]);
        appearanceLog[s[0]]--;
        inStack[s[0]] = true;

        for (int i = 1; i < s.size(); i++) {
            char curChar = s[i], topChar = charStack.top();
            appearanceLog[curChar]--;
            if (inStack[curChar])
                continue;

            while (appearanceLog[topChar] > 0 && curChar < topChar) {
                charStack.pop();
                inStack[topChar] = false;
                if (!charStack.empty())
                    topChar = charStack.top();
                else
                    break;
            }
            charStack.push(curChar);
            inStack[curChar] = true;
        }

        string res = "";
        while (!charStack.empty()) {
            res += charStack.top();
            charStack.pop();
        }
        reverse(res.begin(), res.end());
        return res;
    }
};
