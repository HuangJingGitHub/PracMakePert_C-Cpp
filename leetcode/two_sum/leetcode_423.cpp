class Solution {
public:
    string originalDigits(string s) {
        vector<char> keyChar = {'z', 'w', 'x', 's', 'g', 't', 'v', 'f', 'o', 'i'};
        vector<string> mapStr = {"zero", "two", "six", "seven", "eight", "three", "five", "four", "one", "nine"};
        vector<int> mapNum = {0, 2, 6, 7, 8, 3, 5, 4, 1, 9};
        unordered_map<int, int> numFrequency;

        unordered_map<char, int> charFrequency;
        for (int i = 0; i < s.size(); i++)
            charFrequency[s[i]]++;

        for (int i = 0; i < keyChar.size(); i++) {
            char curKeyChar = keyChar[i];
            int numTimes = charFrequency[curKeyChar];
            numFrequency[mapNum[i]] = numTimes;
            for (char c : mapStr[i])
                charFrequency[c] -= numTimes;
        }        
        
        string res;
        for (int i = 0; i <= 9; i++)
            if (numFrequency.find(i) != numFrequency.end())
                for (int j = 0; j < numFrequency[i]; j++)
                    res += to_string(i);
        return res;
    }
};
