class Solution {
public:
    string originalDigits(string s) {
        vector<int> nums;
        vector<char> keyChar = {'z', 'w', 'x', 's', 'g', 't', 'v', 'f', 'o', 'i'};
        vector<string> mapStr = {"zero", "two", "six", "seven", "eight", "three", "five", "four", "one", "nine"};
        vector<int> mapNum = {0, 2, 6, 7, 8, 3, 5, 4, 1, 9};

        unordered_map<char, int> charFrequency;
        for (int i = 0; i < s.size(); i++)
            charFrequency[s[i]]++;

        for (int i = 0; i < keyChar.size(); i++) {
            char curKeyChar = keyChar[i];

            int numTimes = charFrequency[curKeyChar];
            for (int j = 0; j < numTimes; j++)
                nums.push_back(mapNum[i]);
            for (char c : mapStr[i])
                charFrequency[c] -= numTimes;
        }        

        sort(nums.begin(), nums.end());
        
        string res = "";
        for (int& num : nums)
            res += to_string(num);
        return res;
    }
};
