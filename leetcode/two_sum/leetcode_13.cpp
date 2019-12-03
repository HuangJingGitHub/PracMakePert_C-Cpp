#include <iostream>
#include <vector>
#include <string.h>

using namespace std;

class Solution_self{
    public:
    int romanToInt(string s)
    {
        vector<string> romanStr = { "M", "CM", "D", "CD", "C", "XC", "L", "XL", "X", 
                                    "IX", "V", "IV", "I"};
        vector<int> values = {1000, 900, 500, 400, 100, 90, 50, 40, 10, 9, 5, 4, 1};

        int resultInt = 0;
        for (int i = 0; i < romanStr.size(); i++)
        {
            int j = romanStr[i].size();
            while (s.substr(0, j) == romanStr[i])
            {
                resultInt += values[i];
                s = s.substr(j, s.size());
                cout << "result = " << resultInt << " s = " << s << endl;
            }
        }

        return resultInt;
    }
};

class Solution_14 {
public:
    string longestCommonPrefix(vector<string>& strs) {
        if (strs.size() == 0)
            return "";
        else if (strs.size() == 1)
            return strs[0];
        
        int i, j = 0;
        while (true)
        {   
            for (i = 0; i < strs.size() - 1; i++)
            {
                if (j == strs[i].size() || j == strs[i+1].size())
                    break;

                if (strs[i][j] == strs[i+1][j])
                    continue;
                else
                    break;
            }
            if (i == strs.size() - 1)
                j++;
            else
                break;
        }
        return strs[0].substr(0, j);
    }
};

int main()
{
    Solution_self solRef;
    cout << solRef.romanToInt("V");
}
