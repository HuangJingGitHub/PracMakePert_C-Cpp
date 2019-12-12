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

class Solution {
public:
    vector<vector<int>> threeSum(vector<int>& nums) {
        sort(nums.begin(), nums.end());
        vector<vector<int>> result;

        for(int i = 0; i < nums.size() - 2; i++)
        {
            if (nums[i] > 0)
                break;

            int first = nums[i];
            int leftPointer = i + 1, rightPointer = nums.size() - 1;
            while (leftPointer < rightPointer)
            {
                cout << nums[i] << "+" << nums[leftPointer] << "+" << nums[rightPointer] << endl;
                if (nums[leftPointer] + nums[rightPointer] + nums[first] < 0){
                    leftPointer++;
                    cout << nums[i] << "+" << nums[leftPointer] << "+" << nums[rightPointer] << endl;
                }
                else if (nums[leftPointer] + nums[rightPointer] + nums[first] > 0){
                    rightPointer--;
                }

                else if (nums[leftPointer] + nums[rightPointer] + nums[first] == 0)
                {
                    vector<int> triplet{nums[first], nums[leftPointer], nums[rightPointer]};
                    result.push_back(triplet);
                    leftPointer++;
                    rightPointer--;
                    //while (nums[leftPointer] == nums[leftPointer+1])
                    //    leftPointer++;
                    //while (nums[leftPointer] == nums[leftPointer-1])
                    //    rightPointer--;
                }
                cout << nums[i] << "+" << nums[leftPointer] << "+" << nums[rightPointer] << endl;
            }
        }
    return result;    
    }
};