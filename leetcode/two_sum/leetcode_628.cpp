class Solution {
public:
    int maximumProduct(vector<int>& nums) {
        sort(nums.begin(), nums.end());

        int len = nums.size();
        int temp1 = nums[0] * nums[1] * nums.back(), 
            temp2 = nums[len - 1] * nums[len - 2] * nums[len - 3];
        if (temp1 >= temp2)
            return temp1;
        else
            return temp2;
    }
};
