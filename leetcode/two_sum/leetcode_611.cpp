class Solution {
public:
    int triangleNumber(vector<int>& nums) {
        int res = 0, n = nums.size();
        if (n < 3)
            return 0;

        sort(nums.begin(), nums.end());
        for (int idx_0 = 0; idx_0 < n - 2; idx_0++) {
            for (int idx_1 = idx_0 + 1; idx_1 < n - 1; idx_1++) {
                for (int idx_2 = idx_1 + 1; idx_2 < n; idx_2++) {
                    if (nums[idx_0] + nums[idx_1] > nums[idx_2]) 
                        res++;
                    else 
                        break;
                }
            }
        }
        return res;
    }
};

// Two pointers
class Solution {
public:
    int triangleNumber(vector<int>& nums) {
        int res = 0, n = nums.size();
        if (n < 3)
            return 0;

        sort(nums.begin(), nums.end());
        for (int max_idx = n - 1; max_idx >= 2; max_idx--) {
            int left = 0, right = max_idx - 1;
            while (left < right) {
                if (nums[left] + nums[right] > nums[max_idx]) {
                    res += right - left;
                    right--;
                }
                else {
                    left++;
                }
            }
        }
        return res;
    }
};
