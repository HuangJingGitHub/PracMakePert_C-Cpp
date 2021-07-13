class Solution {
public:
    bool wordPattern(string pattern, string s) {
        unordered_map<char, string> mapLog;
        unordered_map<string, char> reverseLog;
        int wordStart = 0, wordEnd = 0;

        for (int i = 0; i < pattern.size(); i++) {
            char keyChar = pattern[i];
            string valueStr;

            if (wordStart > s.size())  // s too short for pattern
                return false;
            while (wordEnd < s.size() && s[wordEnd] != ' ')
                wordEnd++;
            if (wordEnd < s.size())
                valueStr = s.substr(wordStart, wordEnd - wordStart);
            else
                valueStr = s.substr(wordStart);
            wordStart = ++wordEnd;

            if (mapLog.find(keyChar) == mapLog.end() && reverseLog.find(valueStr) == reverseLog.end()) {
                mapLog[keyChar] = valueStr;
                reverseLog[valueStr] = keyChar;
            }
            else if (mapLog[keyChar] != valueStr || reverseLog[valueStr] != keyChar)
                    return false;
        }
        if (wordEnd != s.size() + 1)  // pattern too short for s
            return false;
        return true;
    }
};
