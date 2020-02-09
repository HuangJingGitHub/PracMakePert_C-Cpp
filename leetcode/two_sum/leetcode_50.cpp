// Learn about fast power function by recursion.
class Solution {
public:
    double myPow(double x, int n) {
        long long N = n;
        if (n < 0){
            x = 1 / x;
            return fastPow(x, -N);
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
