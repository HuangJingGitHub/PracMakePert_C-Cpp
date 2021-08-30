// Mian problem is how to avoid situations like [1, 1, 2, 2, 3, 3] ---> [1, 3, 1, 3, 2, 2]
class Solution {
public:
    void wiggleSort(vector<int>& nums) {
        vector<int> temp(nums);
        sort(temp.begin(), temp.end());

        int leftPtr = nums.size() / 2, rightPtr = nums.size() - 1;
        if (nums.size() % 2 == 0)  // Size of nums is even.
            leftPtr--;
        int minRight = leftPtr + 1, idx = 0;
        while (leftPtr >= 0 && rightPtr >= minRight) {
            nums[idx++] = temp[leftPtr];
            nums[idx++] = temp[rightPtr];
            leftPtr--;
            rightPtr--;
        }

        if (leftPtr == 0)  // Size of nums is odd.
            nums.back() = temp.front();
    }
};
