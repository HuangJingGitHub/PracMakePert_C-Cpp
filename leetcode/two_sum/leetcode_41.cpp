// An interesting problem, refer to the solutions and discussions on leetcode.
class Solution {
public:
    int firstMissingPositive(vector<int>& nums) {
        for (int i = 0; i < nums.size(); i++) {
            if (nums[i] != i + 1) {
                while (nums[i] != i + 1) {
                    if (nums[i] <= 0 || nums[i] > nums.size() || nums[i] == nums[nums[i] - 1])
                        break;
                    swap(nums[i], nums[nums[i] - 1]);
                }
            }
        }

        for (int i = 0; i < nums.size(); i++)
            if (nums[i] != i + 1)
                return i + 1;
        return nums.size() + 1;
    }
};
