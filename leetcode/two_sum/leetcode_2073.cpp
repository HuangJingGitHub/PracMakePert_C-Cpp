class Solution {
public:
    int mergeSort(vector<int>& nums, vector<int>& temp, int leftPtr, int rightPtr) {
        if (leftPtr >= rightPtr)
            return 0;
        
        int mid = leftPtr + (rightPtr - leftPtr) / 2;
        int reverseCount = mergeSort(nums, temp, leftPtr, mid) + mergeSort(nums, temp, mid + 1, rightPtr);
        int i = leftPtr, j = mid + 1, pos = leftPtr;
        
        while (i <= mid && j <= rightPtr) {
            if (nums[i] <= nums[j]) {
                temp[pos] = nums[i];
                i++;
                reverseCount += (j - mid - 1);
            }
            else {
                temp[pos] = nums[j];
                ++j;
            }
            pos++;
        }  
        for (int k = i; k <= mid; k++) {
            temp[pos++] = nums[k];
            reverseCount += (j - mid - 1);
        }
        for (int k = j; k <= rightPtr; k++) {
            temp[pos++] = nums[k];
        }
        copy(temp.begin() + leftPtr, temp.begin() + rightPtr + 1, nums.begin() + leftPtr);
        return reverseCount;
    }

    int reversePairs(vector<int>& nums) {
        int n = nums.size();
        vector<int> temp(n);
        return mergeSort(nums, temp, 0, n - 1);
    }
};
