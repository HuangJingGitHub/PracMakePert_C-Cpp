class Solution {
public:
    int longestOnes(vector<int>& nums, int k) {
        int res = 0, left = 0, right = 0, zeroNum = 0;

        while (right < nums.size()) {
            if (nums[right] == 0)
                zeroNum++;
            while (zeroNum > k) {
                if (nums[left] == 0)
                    zeroNum--;
                left++;
            }
            res = max(res, right - left + 1);
            right++;
        }
        return res;
    }
};
