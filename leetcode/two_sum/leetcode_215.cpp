class Solution {
public:
    int findKthLargest(vector<int>& nums, int k) {
        int left = 0, right = nums.size() - 1, targetIdx = nums.size() - k;
        while (true) {
            int curSortedIdx = partition(nums, left, right);
            if (curSortedIdx == targetIdx)
                return nums[curSortedIdx];
            else if (curSortedIdx < targetIdx)
                left = curSortedIdx + 1;
            else
                right = curSortedIdx - 1;
        }
    }

    // *very classical partition algorithm
    int partition(vector<int>& nums, int left, int right) {
        int povit = nums[left];
        int j = left;
        for (int i = left + 1; i <= right; i++) {
            if (nums[i] <= povit) {
                j++;
                swap(nums[j], nums[i]);
            }
        }
        swap(nums[left], nums[j]);
        return j;
    }
};
