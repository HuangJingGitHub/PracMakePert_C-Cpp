#include <map>
#include <vector>
#include <iostream>
#include <string.h>

using namespace std;

class Solution {
public:
    string intToRoman(int num) {
        std::map<int, string> intToRomanMap;
        intToRomanMap[1] = "I";
        intToRomanMap[5] = "V";
        intToRomanMap[10] = "X";
        intToRomanMap[50] = "L";
        intToRomanMap[100] = "C";
        intToRomanMap[500] = "D";
        intToRomanMap[1000] = "M";
        
        string tempStr;
        for(int i = 1; i < 9; i++)
        {
            if (i < 3)
            {   tempStr = intToRomanMap[i];
                intToRomanMap[i+1] = tempStr.append(intToRomanMap[1]);
                tempStr = intToRomanMap[i*10];
                intToRomanMap[(i+1)*10] = tempStr.append(intToRomanMap[10]);
                tempStr = intToRomanMap[i*100];
                intToRomanMap[(i+1)*100] = tempStr.append(intToRomanMap[100]);
                tempStr = intToRomanMap[i*1000];
                intToRomanMap[(i+1)*1000] = tempStr.append(intToRomanMap[1000]);
            }
            else if (i == 3)
            {
                tempStr = intToRomanMap[1];
                intToRomanMap[i+1] = tempStr.append(intToRomanMap[5]);
                tempStr = intToRomanMap[10];
                intToRomanMap[(i+1)*10] = tempStr.append(intToRomanMap[50]);
                tempStr = intToRomanMap[100];
                intToRomanMap[(i+1)*100] = tempStr.append(intToRomanMap[500]);            
            }
            else if (i == 4)    continue;
            else if (i > 4 && i < 8 )
            {
                tempStr = intToRomanMap[5];
                intToRomanMap[i+1] = tempStr.append(intToRomanMap[i-4]);
                tempStr = intToRomanMap[50];
                intToRomanMap[(i+1)*10] = tempStr.append(intToRomanMap[(i-4)*10]);
                tempStr = intToRomanMap[500];
                intToRomanMap[(i+1)*100] = tempStr.append(intToRomanMap[(i-4)*100]);  
            }
            if (i == 8)
            {
                tempStr = intToRomanMap[1];
                intToRomanMap[i+1] = tempStr.append(intToRomanMap[10]);
                tempStr = intToRomanMap[10];
                intToRomanMap[(i+1)*10] = tempStr.append(intToRomanMap[100]);
                tempStr = intToRomanMap[100];
                intToRomanMap[(i+1)*100] = tempStr.append(intToRomanMap[1000]);  
            }
        }
       for (int i = 1; i < 10; i++)
        {
            cout << intToRomanMap[i] << " " << intToRomanMap[i*10] << " " << intToRomanMap[i*100] << endl;
        }

        int dec = 10, remainder = num % dec;
        vector<int> digits{remainder};
        while(num-remainder != 0)
        {
            num -= remainder;
            dec *= 10;
            remainder = num % dec;
            cout << "remainder: " << remainder << endl;
            digits.insert(digits.begin(), remainder);
        }

        string romanStr = intToRomanMap[digits[0]];
        cout << digits[0] << " ";
        for (int i = 1; i < digits.size(); i++)
        {
            cout << digits[i] << " ";
            romanStr.append(intToRomanMap[digits[i]]);
        }

        return romanStr;
    }
};

int main()
{
    Solution sol;
    cout << sol.intToRoman(1434);
}