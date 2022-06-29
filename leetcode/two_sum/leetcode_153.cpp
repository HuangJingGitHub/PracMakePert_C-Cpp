// binary search, but the specific comparison condition setting can be subtle.
class Solution {
public:
    int findMin(vector<int>& nums) {
        int left = 0, right = nums.size() - 1, mid = 0;
        while (left < right) {
            mid = left + (right - left) / 2;
            if (nums[left] <= nums[mid] && nums[mid] < nums[right])
                right = mid;
            else if (nums[left] <= nums[mid] && nums[mid] > nums[right])
                left = mid + 1;
            else if (nums[left] >= nums[mid] && nums[mid] < nums[right])
                right = mid;
        }
        return nums[left];
    }
};


// More compact version, see how the range change to guarantee the result always lies in range [left, right]
class Solution {
public:
    int findMin(vector<int>& nums) {
        int left = 0, right = nums.size() - 1, mid = 0;
        while (left < right) {
            mid = left + (right - left) / 2;
            if (nums[mid] < nums[right])
                right = mid;
            else
                left = mid + 1;
        }
        return nums[left];
    }
};

// or
class Solution {
public:
    int findMin(vector<int>& nums) {
        int left = 0, right = nums.size() - 1, mid = 0;
        while (left <= right) {
            mid = left + (right - left) / 2;
            if (nums[mid] > nums[right])
                left = mid + 1;
            else 
                right--;
        }
        return nums[mid];
    }
};
