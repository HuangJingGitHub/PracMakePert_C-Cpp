class Solution {
public:
    int minMoves2(vector<int>& nums) {
        int res = 0;
        sort(nums.begin(), nums.end());

        for (int i = 0, j = nums.size() - 1; i < j; i++, j--)
            res += nums[j] - nums[i];
        return res;
    }
};
