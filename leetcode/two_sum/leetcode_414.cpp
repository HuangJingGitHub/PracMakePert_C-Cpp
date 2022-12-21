class Solution {
public:
    int thirdMax(vector<int>& nums) {
        if (nums.size() == 1)
            return nums[0];
        else if (nums.size() == 2)
            return max(nums[0], nums[1]);
        
        vector<long int> maxThree(3, LONG_MIN);
        for (int i = 0; i < nums.size(); i++) {
            if (nums[i] > maxThree[0]) {
                maxThree[2] = maxThree[1];
                maxThree[1] = maxThree[0];
                maxThree[0] = nums[i];
            }
            else if (nums[i] > maxThree[1] && nums[i] != maxThree[0]) {
                maxThree[2] = maxThree[1];
                maxThree[1] = nums[i];
            }
            else if (nums[i] > maxThree[2] && nums[i] != maxThree[0] && nums[i] != maxThree[1]) {
                maxThree[2] = nums[i];
            }
        }

        if (maxThree[2] == LONG_MIN)
            return maxThree[0];
        return maxThree[2];
    }
};
