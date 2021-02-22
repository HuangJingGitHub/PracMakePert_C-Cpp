class Solution {
public:
    int dominantIndex(vector<int>& nums) {
        if (nums.size() == 1)
            return 0;
            
        int maxNum = max(nums[0], nums[1]), subMax = min(nums[0], nums[1]), res = 0;
        if (nums[1] > nums[0])
            res = 1;

        for (int i = 2; i < nums.size(); i++) {
            if (nums[i] > maxNum) {
                subMax = maxNum;
                maxNum = nums[i];
                res = i;
            }
            else if (nums[i] > subMax) {
                subMax = nums[i];
            }
        }

        if (maxNum >= 2 * subMax)
            return res;
        else
            return -1;
    }
};
