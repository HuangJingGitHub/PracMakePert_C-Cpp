class Solution {
public:
    vector<int> searchRange(vector<int>& nums, int target) {
        vector<int> ret{-1, -1};
        if (nums.size() == 0)
            return ret;

        int left = 0, right = nums.size() - 1, mid;
        while (left <= right){
            mid = left + (right - left) / 2;
            if (nums[left] == target){
                ret[0] = left;
                break;
            }    
            if (nums[mid] > target)
                right = mid - 1;
            else if (nums[mid] < target)
                left = mid + 1;
            else if (nums[mid] == target){
                right = mid;
                if (left == mid - 1){
                    ret[0] = mid;
                    break;
                }
            }      
        }

        right = nums.size() - 1;
        while (left <= right){
            mid = left + (right - left) / 2;
            if (nums[right] == target){
                ret[1] = right;
                break;
            }    
            if (nums[mid] > target)
                right = mid - 1;
            else if (nums[mid] < target)
                left = mid + 1;
            else if (nums[mid] == target){
                left = mid;
                if (right == mid + 1){
                    ret[1] = mid;
                    break;                    
                }
            }
        }        
        return ret;
    }
};
