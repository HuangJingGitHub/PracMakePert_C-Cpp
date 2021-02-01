// very cool solution
// Note that as long as m, n have different bit numbers at some bit, range AND result at this bit is 0.
class Solution {
public:
    int rangeBitwiseAnd(int m, int n) {
        int shift = 0;
        while (m != n) {
            m >>= 1;
            n >>= 1;
            shift++;
        }
        return m << shift;
    }
};
