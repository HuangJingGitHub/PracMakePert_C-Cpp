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

// or
class Solution {
public:
    int maxSubArray(vector<int>& nums) {
        int res = nums[0], curSum = 0;
        for (int i = 0; i < nums.size(); i++) {
            curSum  += nums[i];
            res = max(res, curSum);
            if (curSum < 0)
                curSum = 0;
        }
        return res;
    }
};

// dp
class Solution {
public:
    int maxSubArray(vector<int>& nums) {
        int res = nums[0], curSum = nums[0];
        vector<int> dp(nums.size(), 0);
        dp[0] = nums[0];
        for (int i = 1; i < nums.size(); i++) {
            dp[i] = max(dp[i - 1] + nums[i], nums[i]);
            res = max(res, dp[i]);
        }
        return res;
    }
};
