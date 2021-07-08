class Solution {
public:
    void moveZeroes(vector<int>& nums) {
        int endNonZeroIdx = nums.size() - 1;
        while (endNonZeroIdx >= 0 && nums[endNonZeroIdx] == 0)
            endNonZeroIdx--;
        
        if (endNonZeroIdx == 0)
            return;

        for (int i = endNonZeroIdx - 1; i >= 0; i--) {
            if (nums[i] == 0) {
                int startIdx = i;
                while (startIdx < endNonZeroIdx) {
                    swap(nums[startIdx], nums[startIdx + 1]);
                    startIdx++;
                }
                endNonZeroIdx--;
            }
        }
    }
};
