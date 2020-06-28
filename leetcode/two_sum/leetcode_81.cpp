// Refer to problem 33 without duplicate entries. Just add one extra branch.
class Solution {
public:
    bool search(vector<int>& nums, int target) {
        if (nums.empty())
            return false;
        
        int left = 0, right = nums.size() - 1, mid;
        while (left <= right){
            mid = (left + right) / 2;

            if (nums[mid] == target)
                return true;
            
            if (nums[left] < nums[mid]){
                if (nums[left] <= target && target < nums[mid])
                    right = mid - 1;
                else
                    left = mid + 1;
            }
            else if (nums[left] == nums[mid]){
                left++;
            }
            else{
                if (nums[mid] < target && target <= nums[right])
                    left = mid + 1;
                else
                    right = mid - 1;
            }
        }
        return false;
    }
};