class Solution {
public:
    int findUnsortedSubarray(vector<int>& nums) {
        int mid_max = nums[0], mid_min = nums.back();  // mid-middle range
        int mid_start = 0, mid_end = 0;

        for (int i = 0; i < nums.size(); i++) {
            if (nums[i] < mid_max)
                mid_end = i;
            else
                mid_max = nums[i];
        }
        
        for (int i = nums.size() - 1; i >= 0; i--) {
            if (nums[i] > mid_min)  
                mid_start = i;
            else
                mid_min = nums[i];
        } 

        if (mid_start == 0 && mid_end == 0)
            return 0;
        return mid_end - mid_start + 1;
    }
};
