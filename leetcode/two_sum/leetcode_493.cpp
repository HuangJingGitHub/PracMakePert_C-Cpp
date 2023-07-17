class Solution {
public:
    int reversePairs(vector<int>& nums) {
        if (nums.size() < 2)
            return 0;
        
        int res = 0;
        mergeSort(nums, 0, nums.size() - 1, res);
        return res;
    }

    void mergeSort(vector<int>& nums, int start, int end, int& count) {
        if (start == end)
            return;
        
        int mid = start + (end - start) / 2;
        mergeSort(nums, start, mid, count);
        mergeSort(nums, mid + 1, end, count);
        
        int i = start, j = mid + 1;
        while (i <= mid && j <= end) {
            if ((long)nums[i] > 2 * (long)nums[j]) {
                count += mid - i + 1;
                j++;
            }
            else 
                i++;
        }

        vector<int> temp(end - start + 1, 0);
        i = start;
        j = mid + 1;
        int idx = 0;
        while (i <= mid && j <= end)
            temp[idx++] = nums[i] < nums[j] ? nums[i++] : nums[j++];
        while (i <= mid)
            temp[idx++] = nums[i++];
        while (j <= end)
            temp[idx++] = nums[j++];
        
        for (i = 0, j = start; j <= end; i++, j++)
            nums[j] = temp[i];
    }
};
