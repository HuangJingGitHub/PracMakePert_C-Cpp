// Solution 1
class Solution {
public:
    int maxSubArray(vector<int>& nums) {
        if (nums.size() == 0)
            return NULL;
            
        int res = nums[0];
        int sum = 0;
        for (int num:nums){
            if (sum > 0)
                sum += num;
            else
                sum = num;
            res = max(res, sum);
        }
        return res;
    }
};

// Solution 2
class Solution {
public:
    int maxSubArray(vector<int>& nums) {
        int currentSum = nums[0], maxSum = nums[0];
        
        for (int i = 1; i < nums.size(); i++){
            currentSum = max(currentSum + nums[i], nums[i]);
            maxSum = max(maxSum, currentSum);
        }
        return maxSum;
    }
};
