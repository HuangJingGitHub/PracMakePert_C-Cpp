#include <iostream>
#include <vector>
#include <numeric>
#include <algorithm>

using namespace std;

class Solution {
public:
    int threeSumClosest(vector<int>& nums, int target) {
        sort(nums.begin(), nums.end());
        int ans, smallestDif;
        
        if (nums.size() <= 3)
            return accumulate(nums.begin(), nums.end(), 0);

        ans = nums[0] + nums[1] + nums[nums.size() - 1];
        smallestDif = abs(ans - target);

        for(int i = 0; i < nums.size() - 2; i++)
        {
            if (i > 0 && nums[i] == nums[i-1])
                continue;
            int leftPointer = i + 1, rightPointer = nums.size() - 1;
            
            while (leftPointer < rightPointer)
            {
                int dif = nums[leftPointer] + nums[rightPointer] + nums[i] - target;
                ans = abs(dif) < smallestDif ? dif + target : ans;
                smallestDif = abs(dif) < smallestDif ? abs(dif) : smallestDif;
                
                if ( dif < 0)
                    leftPointer++;
                else if (dif > 0)
                    rightPointer--;               
                else if (dif == 0)
                    return target;               
            }
        }
    return ans;    
    } 
}; 

int main()
{
    Solution sol;
    vector<int> testNums{-10,0,-2,3,-8,1,-10,8,-8,6,-7,0,-7,2,2,-5,-8,1,-4,6};
    int testTarget = 17;
    int testAns = sol.threeSumClosest(testNums, testTarget);
    cout << testAns << endl;
}


class Solution {
public:
    int threeSumClosest(vector<int>& nums, int target) {
        int res = 0, sum, dif = INT_MAX;
        sort(nums.begin(), nums.end());

        for (int i = 0; i <= nums.size() - 3; i++) {
            if (i > 0 && nums[i] == nums[i - 1])
                continue;

            int leftPt = i + 1, rightPt = nums.size() - 1;
            while (leftPt < rightPt) {
                sum = nums[i] + nums[leftPt] + nums[rightPt];
                if (sum == target)
                    return sum;
                if (abs(sum - target) < dif) {
                    dif = abs(sum - target);
                    res = sum;
                }
                if (sum > target) {
                    rightPt--;
                    while (leftPt < rightPt && nums[rightPt] == nums[rightPt + 1])
                        rightPt--;
                }
                else {
                    leftPt++;
                    while (leftPt < rightPt && nums[leftPt] == nums[leftPt - 1])
                        leftPt++;
                }
            }
        }
        return res;
    }
};
