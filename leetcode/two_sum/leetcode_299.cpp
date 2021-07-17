class Solution {
public:
    string getHint(string secret, string guess) {
        unordered_map<char, int> secretLog, guessLog, bullLog;
        int bullNum = 0, cowNum = 0;
        
        for (int i = 0; i < secret.size(); i++) {
            if (secret[i] == guess[i])
                bullNum++;
            else {
                secretLog[secret[i]]++;
                guessLog[guess[i]]++;
            }
        }

        for (auto itr = guessLog.begin(); itr != guessLog.end(); itr++) {
            char curChar = itr->first;
            if (secretLog.find(curChar) == secretLog.end())
                continue;
            else
                cowNum += min(guessLog[curChar], secretLog[curChar]);
        }

        return to_string(bullNum) + "A" + to_string(cowNum) + "B";
    }
};
