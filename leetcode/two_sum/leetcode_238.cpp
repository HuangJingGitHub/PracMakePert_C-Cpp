class Solution {
public:
    vector<int> productExceptSelf(vector<int>& nums) {
        vector<int> res(nums.size());
        int k = 1;
        for (size_t i = 0; i < nums.size(); i++) {
            res[i] = k;
            k *= nums[i];
        }

        k = 1;
        for (int i = nums.size() - 1; i >= 0; i--) {
            res[i] *= k;
            k *= nums[i];
        }
        return res;
    }
};
