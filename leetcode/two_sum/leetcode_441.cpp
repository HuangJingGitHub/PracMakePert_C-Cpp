class Solution {
public:
    int arrangeCoins(int n) {
        long sum = 1, res = 1;

        while (sum <= n) {
            res++;
            sum = res * (res + 1) / 2;
        }

        return res - 1;
    }
};
