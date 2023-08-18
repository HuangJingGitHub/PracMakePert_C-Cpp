class Solution {
public:
    int totalHammingDistance(vector<int>& nums) {
        int res = 0, num_len = nums.size();
        vector<int> bit_cnt(32);  // number of 1 at each bit position

        for (int i = 0; i < nums.size(); i++) {
            for (int bit_idx = 0; bit_idx < 32; bit_idx++) {
                bit_cnt[bit_idx] += nums[i] & 1;
                nums[i] >>= 1;
            }
        }

        for (int i = 0; i < 32; i++)
            res += bit_cnt[i] * (num_len - bit_cnt[i]);
        return res;
    }
};
