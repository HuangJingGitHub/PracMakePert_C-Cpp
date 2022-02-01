class Solution {
public:
    uint32_t reverseBits(uint32_t n) {
       uint32_t res = 0, shift = 32;
       while (shift > 0) {
            res <<= 1;
            res += n & 1;
            n >>= 1;
            shift--;
       }
       return res;
    }
};


class Solution {
public:
    uint32_t reverseBits(uint32_t n) {
        int res = 0;
        for (int i = 1; i < 32; i++) {
            res += (n & 1);
            res <<= 1;
            n >>= 1;
        }
        res += (n & 1);
        return res;
    }
};
