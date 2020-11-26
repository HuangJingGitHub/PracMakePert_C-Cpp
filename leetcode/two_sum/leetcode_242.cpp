class Solution {
public:
    bool isAnagram(string s, string t) {
        if (s.size() != t.size())
            return false;

        unordered_map<char, int> charLog;
        for (char c : s)
            charLog[c]++;
        for (char c : t)
            charLog[c]--;
        for (auto itr = charLog.begin(); itr != charLog.end(); itr++)
            if (itr->second != 0)
                return false;
        return true;
    }
};
