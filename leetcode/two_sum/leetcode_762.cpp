class Solution {
public:
    int countPrimeSetBits(int left, int right) {
        int res = 0;
        set<int> primeSet{2, 3, 5, 7, 11, 13, 17, 19};

        for (int i = left; i <= right; i++) {
            int setBitNum = bitCount(i);
            if (primeSet.find(setBitNum) != primeSet.end())
                res++;
        }
        return res;
    }

    int bitCount(unsigned x) {
        x = x - ((x >> 1) & 0x55555555);
        x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
        x = (x + (x >> 4)) & 0x0F0F0F0F;
        x = x + (x >> 8);
        x = x + (x >> 16);
        return x & 0x0000003F;
    }    
};
