// ***The solution path is quite interesting***. See the offical and the leetcode 100 blog for reference.
class Solution {
public:
    void nextPermutation(vector<int>& nums) {
        if (nums.size() <= 1)
            return;

        int i;
        for (i = nums.size()-1; i > 0; i--)
        {
            if (nums[i-1] < nums[i])
            {
                for (int j = nums.size()-1; j >= i; j--)
                {
                    if (nums[j] > nums[i-1])
                    {
                        int temp = nums[i-1];
                        nums[i-1] = nums[j];
                        nums[j] = temp;
                        for (j = nums.size()-1; i <= j; i++, j--)
                        {
                            temp = nums[i];
                            nums[i] = nums[j];
                            nums[j] = temp;
                        }
                    }
                }
                break;
            }
        }
        
        if (i == 0)
        for (int j = nums.size()-1; i <= j; i++, j--)
        {
            int temp = nums[i];
            nums[i] = nums[j];
            nums[j] = temp;
        }
    }
};
