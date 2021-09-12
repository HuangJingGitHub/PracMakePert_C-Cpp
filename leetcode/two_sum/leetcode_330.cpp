class Solution {
public:
    int minPatches(vector<int>& nums, int n) {
        long curRange = 0;
        int numsSize = nums.size();
        int res = 0;

        for (long i = 1, pos = 0; i <= n; ) {
            if (pos >= numsSize || i < nums[pos]) {
                res++;
                curRange += i;
            }
            else {
                curRange += nums[pos];
                pos++;
            }
            i = curRange + 1;
        }

        return res;
    }
};
