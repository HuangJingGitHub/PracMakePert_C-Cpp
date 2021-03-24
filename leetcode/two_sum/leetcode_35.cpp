class Solution {
public:
    int searchInsert(vector<int>& nums, int target) {
        int left = 0, right = nums.size() - 1, mid;
        while(left <= right) {
            mid = left + (right - left) / 2;
            if (nums[mid] == target)
                return mid;
            else if (nums[mid] < target)
                left = mid + 1;
            else 
                right = mid - 1;
        }
        return left;  // Note left is always the right idx target should be inserted no matter what case the binary search ends with.
    }                 // If it connot find the target, then binary search will always end in a subrange of length 1 or 2!!!
};
