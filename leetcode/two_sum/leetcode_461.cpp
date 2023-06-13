class Solution {
public:
    int hammingDistance(int x, int y) {
        int res = 0, orRes = x | y;
        for (int i = 0; i < 32; i++) {
            res += (orRes & 1);
            orRes = orRes >> 1;
    }
    return res;
};
