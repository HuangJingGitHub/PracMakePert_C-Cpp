// will exceed time limit
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

//  Use factor to delete non-prime number, just linear time.
class Solution {
public:
    int countPrimes(int n) {
        if (n <= 2)
            return 0;

        vector<bool> isPrime(n - 1, true);
        isPrime[0] = false;

        int res = 0;
        for (int i = 2; i < n; i++) {
            if (isPrime[i - 1]) {
                res++;
                for (int j = 2 * i; j < n; j += i)
                    isPrime[j - 1] = false;
            }
        } 
        return res;
    }
};
