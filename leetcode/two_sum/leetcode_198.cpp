class Solution {
public:
    int rob(vector<int>& nums) {
        if (nums.size() == 0)
            return 0;
        
        int prepreRes = 0, preRes = nums[0], res = nums[0];
        for (int i = 1; i < nums.size(); i++){
            res = max(prepreRes + nums[i], preRes);
            prepreRes = preRes;
            preRes = res;
        }

        return res;
    }
};
