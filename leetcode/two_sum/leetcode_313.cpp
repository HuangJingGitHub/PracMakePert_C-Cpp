class Solution {
public:
    int nthSuperUglyNumber(int n, vector<int>& primes) {
        int res = 1;
        for (int i = 1; i < n; i++) {
            int product = INT_MAX;
            for (int& prime : primes)
                product = min(product, res * prime);
            res = product;
        }
        return res;
    }
};
