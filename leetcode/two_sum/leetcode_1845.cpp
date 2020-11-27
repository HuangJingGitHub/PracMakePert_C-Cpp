class Solution {
public:
    int maxSubArray(vector<int>& nums) {
        int res = INT_MIN, sum = 0, i = 0;
        while (i < nums.size()){
            if (sum < 0)
                sum = 0;
            sum += nums[i];
            if (sum > res)
                res = sum;
            i++;
        }
        return res;
    }
};
