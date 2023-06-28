class Solution {
public:
    vector<string> findWords(vector<string>& words) {
        string firstRowStr = "qwertyuiop",
               secondRowStr = "asdfghjkl",
               thridRowStr = "zxcvbnm";
        unordered_map<char, int> rowNumMap;
        for (char c : firstRowStr)
            rowNumMap[c] = 1;
        for (char c : secondRowStr)
            rowNumMap[c] = 2;
        for (char c : thridRowStr)
            rowNumMap[c] = 3;
        
        for (string& word : words)
    }
};
