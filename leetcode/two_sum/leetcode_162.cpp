// MIT 6.006 (2011), first lecture
class Solution {
public:
    int findPeakElement(vector<int>& nums) {
        return binaryFindPeak(nums, 0, nums.size() - 1);
    }

    int binaryFindPeak(vector<int>& nums, int left, int right){
        if (left == right)
            return left;
        
        int len = right - left + 1;
        if (len % 2 == 0){
            int mid = left + len / 2 - 1;
            if (nums[mid] > nums[mid + 1])
                return binaryFindPeak(nums, left, mid);
            else
                return binaryFindPeak(nums, mid + 1, right);
        }
        else{
            int mid = left + len / 2;
            if (nums[mid] > nums[mid - 1] && nums[mid] > nums[mid + 1])
                return mid;
            else if (nums[mid - 1] > nums[mid + 1])
                return binaryFindPeak(nums, left, mid - 1);
            else
                return binaryFindPeak(nums, mid + 1, right);
        }
    }
};


// Much more compact version
class Solution {
public:
    int findPeakElement(vector<int>& nums) {
        if (nums.size() == 0)
            return 0;

        int left = 0, right = nums.size() - 1, mid;
        // use left < right - 1 instead left < right to avoid discussion cases where left, mid overlap
        while (left < right - 1) {
            mid = left + (right - left) / 2;
            if (nums[mid] > nums[mid - 1] && nums[mid] > nums[mid + 1])
                 return mid;
            if (nums[mid - 1] >= nums[mid + 1])
                right = mid - 1;
            else
                left = mid + 1; 
        }
        return (nums[left] > nums[right]) ? left : right;
    }
};
