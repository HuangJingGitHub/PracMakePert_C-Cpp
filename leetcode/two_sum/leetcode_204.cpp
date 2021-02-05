class Solution {
public:
    int countPrimes(int n) {
        if (n <= 2)
            return 0;
        int res = 0;
        for (int i = 2; i < n; i++)
            if (isPrime(i))
                res++;
        return res;
    }

    bool isPrime(int &num) {
        int bound = (int) sqrt(num) + 1;
        for (int i = 2; i < bound; i++)
            if (num % i == 0)
                return false;
        return true;
    }
};
