class Solution {
public:
    int hammingDistance(int x, int y) {
        int res = 0, xorRes = x ^ y;
        for (int i = 0; i < 32; i++) {
            res += xorRes & 1;
            xorRes = xorRes >> 1;
        }
    return res;
    }
};
