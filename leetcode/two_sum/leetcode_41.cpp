// Interesting problem, refer to the solutions and discussions on leetcode.
class Solution {
public:
    int firstMissingPositive(vector<int>& nums) {
        for (int i = 0; i < nums.size(); i++){
            while (nums[i] != i+1){
                if (nums[i] <= 0 || nums[i] > nums.size() || nums[i] == nums[nums[i] - 1])
                    break;
                int idx = nums[i] - 1;
                nums[i] = nums[idx];
                nums[idx] = idx + 1;
            }
        }
        
        for (int i = 0; i < nums.size(); i++){
            if (nums[i] != i+1)
                return i+1;
        }

        return nums.size() + 1;
    }
};
