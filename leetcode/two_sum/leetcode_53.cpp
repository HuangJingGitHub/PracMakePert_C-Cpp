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
