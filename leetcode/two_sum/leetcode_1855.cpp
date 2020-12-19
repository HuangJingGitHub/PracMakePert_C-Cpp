// No sorting solution, twice linear traverse.
class Solution {
public:
    void wiggleSort(vector<int>& nums) {
        for (int i = 0; i + 1 < nums.size(); i += 2)  // One traversal to ensure "one-side peak".
            if (nums[i] < nums[i + 1]) {
                int temp = nums[i];
                nums[i] = nums[i + 1];
                nums[i + 1] = temp;
            }
        for (int i = 1; i + 1 < nums.size(); i += 2) // Another traversal to fix positions that does not have peak-valley sequence on the other side.
            if (nums[i] > nums[i + 1]) {
                int temp = nums[i];
                nums[i] = nums[i + 1];
                nums[i + 1] = temp;
            }
    }
};
