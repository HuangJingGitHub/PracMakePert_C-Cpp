class Solution {
public:
    bool canConstruct(string ransomNote, string magazine) {
        unordered_map<char, int> charDict;
        for (char c : magazine)
            charDict[c]++;
        for (char c : ransomNote) { 
            charDict[c]--;
            if (charDict[c] < 0)
                return false;
        }
        return true;
    }
};
