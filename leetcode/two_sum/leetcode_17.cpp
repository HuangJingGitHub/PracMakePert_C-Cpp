#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <math.h>

using namespace std;

/*class Solution {
public:
    vector<string> letterCombinations(string digits) {
        vector<string> digitsMap{"2", "3", "4", "5", "6", "7", "8", "9"};
        vector<string> lettersMap{"abc", "def", "ghi", "jkl", "mno", "pqrs", "tuv", "wxyz"};
        int ansSize = 1, repeatTimes;
        for (auto x:digits)
        {
            if (x == "7" || x == "9")
                ansSize *= 4;
            else
                ansSize *= 3;  
        }
        repeatTimes = ansSize;
        vector<string> ans(ansSize);

        for(int i = 0; i < digits.size(); i++)
        {
            string currentDigitStr = digits[i];
            int currentDigit = stoi(currentDigitStr);
            string currentLettersStr = lettersMap[currentDigit - 2];
            if (currentDigit == 7 || currentDigit == 9)
                repeatTimes /= 4;
            else
                repeatTimes /= 3;
        }
        
    }
};*/

class Solution_self{
public:
    vector<string> letterCombinations(string digits) {
        // vector<string> digitsMap{"2", "3", "4", "5", "6", "7", "8", "9"};
        vector<string> lettersMap{"abc", "def", "ghi", "jkl", "mno", "pqrs", "tuv", "wxyz"};
        vector<string> ans;

        if (digits.size() > 1)
        {   
            char lastLetter = digits[digits.size() - 1];
            int lastDigit = stoi(&lastLetter), sizeTimes;
            if (lastDigit == 7 or lastDigit == 9)
                sizeTimes = 4;
            else
                sizeTimes = 3;
           
            vector<string> lastAns = this->letterCombinations(digits.substr(0, digits.size() - 1));
            // ans.reserve(lastAns.size() * sizeTimes);
            for(int i = 0; i < sizeTimes; i++)
                ans.insert(ans.end(), lastAns.begin(), lastAns.end());
            for(int i = 0; i < ans.size(); i++)
                ans[i] += lettersMap[i % sizeTimes];
            return ans;
        }

        if (digits.size() == 1)
        {
            int currentDigit = stoi(digits);
            string currentLettersStr = lettersMap[currentDigit - 2];
            cout << currentLettersStr << endl;
            for(int i = 1; i < currentLettersStr.size(); i++)
                ans.push_back(&currentLettersStr[i]);          
            return ans;
        }
    }
};

int main()
{
    Solution_self sol;
    string testNum = "23";
    vector<string> ansTest;
    ansTest = sol.letterCombinations(testNum);

    for (auto x:ansTest)
        cout << x << " ";
}