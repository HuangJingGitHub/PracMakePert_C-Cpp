class Solution {
public:
    int findMin(vector<int>& nums) {
        int left = 0, right = nums.size() - 1, mid;
        while (left < right) {
            mid = left + (right - left) / 2;
            if (nums[mid] < nums[right])
                right = mid;
            else if (nums[mid] == nums[right])   // duplicate case e.g. [3,3,3,3,1,3,3]
                right--;
            else
                left = mid + 1;
        }
        return nums[left];
    }
};
