class Solution {
public:
    int hammingWeight(uint32_t n) {
        int res = 0, bit = 0;
        while (bit < 32) {
            res += n & 1;
            n >>= 1;
            bit++;
        }
        return res;
    }
};
