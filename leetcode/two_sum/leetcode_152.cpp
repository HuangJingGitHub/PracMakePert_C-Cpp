class Solution {
public:
    int maxProduct(vector<int>& nums) {
        int imax = nums[0], imin = nums[0], res = nums[0];
        for (int i = 1; i < nums.size(); i++) {
            if (nums[i] < 0)   // This process is cool.
                swap(imax, imin);
            imax = max(imax * nums[i], nums[i]);
            imin = min(imin * nums[i], nums[i]);
            res = max(imax, res);
        }
        return res;
    }
};
