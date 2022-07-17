#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>
#include <math.h>

using namespace std;

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

// backtrack
class Solution {
public:
    vector<string> letterMap{"abc", "def", "ghi", "jkl", "mno", "pqrs", "tuv", "wxyz"};

    vector<string> letterCombinations(string digits) {
        vector<string> res;
        if (digits.empty())
            return res;
        
        string curStr;
        backtrace(digits, curStr, res, 0);
        return res;
    }

    void backtrack(string& digits, string& curStr, vector<string>& res, int idx) {
        if (idx == digits.size()) {
            res.push_back(curStr);
            return;
        }

        int digit = digits[idx] - '2';
        string buttonStr = letterMap[digit];
        for (int i = 0; i < buttonStr.size(); i++) {
            curStr += buttonStr[i];
            backtrack(digits, curStr, res, idx + 1);
            curStr.pop_back();
        }
    }
};


class Solution {
public:
    vector<string> buttonStr{"", "", "abc", "def", "ghi", "jkl", "mno", "pqrs", "tuv", "wxyz"};

    vector<string> letterCombinations(string digits) {
        vector<string> res;

        if (digits.size() == 0)
            return {};
        if (digits.size() == 1) {
            int curDigit = stoi(digits);
            string curStr = buttonStr[curDigit];
            for (int i = 0; i < curStr.size(); i++) {
                string tempStr = "";
                tempStr.push_back(curStr[i]);
                res.push_back(tempStr);
            }
            return res;
        }

        vector<string> preRes = letterCombinations(digits.substr(0, digits.size() - 1));
        int endDigit = digits.back() - '0';
        string endStr = buttonStr[endDigit];
        for (int i = 0; i < endStr.size(); i++) 
            for (int j = 0; j < preRes.size(); j++) {
                string tempStr = preRes[j];
                tempStr.push_back(endStr[i]);
                res.push_back(tempStr);
            }
        
        return res;
    }
};
