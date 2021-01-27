class Solution {
public:
    void rotate(vector<int>& nums, int k) {
        // O(1) space, but O(n^2) time, will exceed the time limit
        /*k %= nums.size();
        for (int i = nums.size() - k; i < nums.size(); i++)
            for (int j = i - 1; i - j <= nums.size() - k; j--)
                swap(nums[j], nums[j + 1]); */
        
        k %= nums.size();
        vector<int> numsCopy = nums;
        for (int i = 0; i < k; i++)
            nums[i] = numsCopy[nums.size() - k + i];
        for (int i = k; i < nums.size(); i++)
            nums[i] = numsCopy[i - k];
    } 
};
