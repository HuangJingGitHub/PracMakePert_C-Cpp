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

int main()
{
    Solution_self solRef;
    cout << solRef.romanToInt("V");
}