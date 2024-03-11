class Solution {
public:
    // Note the difference of indices of number pairs before and after the single number: even-odd ---> odd-even
    // Details are important.
    int singleNonDuplicate(vector<int>& nums) {
        int left = 0, right = nums.size() - 1, mid;
        
        while (left < right) {
            mid = left + (right - left) / 2;
            // cout << left << "-" << right << "-" << mid << "\n";

            if (mid % 2 == 0) {
                if (mid - 1 >= 0 && nums[mid] == nums[mid - 1]) 
                    right = mid - 2;
                else if (mid + 1 < nums.size() && nums[mid] == nums[mid + 1])
                    left = mid + 2;
                else
                    return nums[mid];
            }
            else {
                if (mid - 1 >= 0 && nums[mid] == nums[mid - 1]) 
                    left = mid + 1;
                else if (mid + 1 < nums.size() && nums[mid] == nums[mid + 1])
                    right = mid - 1;
                else
                    return nums[mid];
            }            
        }

        return nums[left];
    }
};
