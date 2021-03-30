class Solution {
public:
    int minSubArrayLen(int target, vector<int>& nums) {
        int left = 0, right = 0, sum = nums[0], res = 0;
      
        while (right < nums.size()) {
            if (sum < target) {
                right++;
                if (right == nums.size())
                    return res;
                sum += nums[right];
            }
            else {
                if (res == 0)
                    res = right - left + 1;
                else    
                    res = min(res, right - left + 1);
                if (res == 1)
                    return 1;
                sum -= nums[left];
                left++;
            }
        }
        return res;
    }
};
