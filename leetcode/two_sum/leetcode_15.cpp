#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class Solution {
public:
    vector<vector<int>> threeSum(vector<int>& nums) {
        sort(nums.begin(), nums.end());
        vector<vector<int>> result;
        
        if (nums.size() < 3)
            return result;

        for(int i = 0; i < nums.size() - 2; i++)
        {
            if (nums[i] > 0)
                break;
            if (i > 0 && nums[i] == nums[i-1])
                continue;
            int leftPointer = i + 1, rightPointer = nums.size() - 1;
            
            while (leftPointer < rightPointer)
            {
                int sum = nums[leftPointer] + nums[rightPointer] + nums[i];
                if ( sum < 0){
                    leftPointer++;
                }
                else if (sum > 0){
                    rightPointer--;
                }
                
                else if (sum == 0)
                {
                    vector<int> triplet{nums[i], nums[leftPointer], nums[rightPointer]};
                    result.push_back(triplet);
                    leftPointer++;
                    rightPointer--;
                    while (leftPointer < rightPointer && nums[leftPointer] == nums[leftPointer-1])
                        leftPointer++;
                    while (leftPointer < rightPointer && nums[rightPointer] == nums[rightPointer+1])
                        rightPointer--;
                }
            }
        }
    return result;    
    }
};

int main()
{
    Solution sol;
    vector<int> numsTest{-4, -1, -1, 2, 2, 3, 0};
    vector<vector<int>> resultTest = sol.threeSum(numsTest);

    for(auto x:resultTest)
    {
        for(int i=0; i<3; i++)
            cout << x[i] << " ";
        cout << endl;
    }
}