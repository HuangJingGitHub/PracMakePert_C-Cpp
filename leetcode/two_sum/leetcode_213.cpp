class Solution {
public:
    int rob(vector<int>& nums) {
        if (nums.size() == 1)
            return nums[0];
            
        int prepreRes = nums[0], preRes = nums[0], res0 = nums[0], res1 = nums[1];
        // House 1 is robbed
        for (int i = 2; i < nums.size() - 1; i++) {
            res0 = max(prepreRes + nums[i], preRes);
            prepreRes = preRes;
            preRes = res0;
        }
    
        // House 1 is not robbed.
        prepreRes = 0;
        preRes = nums[1];
        for (int i = 2; i < nums.size(); i++) {
            res1 = max(prepreRes + nums[i], preRes);
            prepreRes = preRes;
            preRes = res1;
        }

        return max(res0, res1);
    }
};
