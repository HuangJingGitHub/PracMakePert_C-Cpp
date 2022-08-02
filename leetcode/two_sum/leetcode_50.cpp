// Learn about fast power function by recursion.
class Solution {
public:
    double myPow(double x, int n) {
        long long N = n;
        if (n < 0){
            x = 1 / x;
            return fastPow(x, -N);  // or return 1 / fastPow(x, -N);
        }
        return fastPow(x, N);
    }
    
    double fastPow(double x, long long n){
        if (n == 0)
            return 1.0;
            
        double half = fastPow(x, n/2);
        if (n % 2 == 0)
            return half * half;
        else 
            return half * half * x;
    }
};

class Solution {
public:
    double myPow(double x, int n) {
        long N = n;
        if (N < 0)
            return 1 / myPowLong(x, -N);
        return myPowLong(x, N);
    }

    double myPowLong(double x, long N) {
        if (N == 0)
            return 1;
        
        double res = 1;
        if (N % 2 == 0) {
            double halfRes = myPow(x, N / 2);
            res = halfRes * halfRes;
        }
        else if (N % 2 == 1) {
            double halfRes = myPow(x, N / 2);
            res = halfRes * halfRes * x;
        }
        return res;
    }
};
