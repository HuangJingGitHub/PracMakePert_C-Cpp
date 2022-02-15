class Solution {
public:
    int divide(int dividend, int divisor) {
        int INT_MIN_32 = -pow(2,31), INT_MAX_32 = pow(2,31) - 1;
        if (dividend == INT_MIN_32 && divisor == -1)
            return INT_MAX_32;
        if (dividend == INT_MIN_32 && divisor == 1)
            return INT_MIN_32;
        if (dividend == INT_MAX_32 && divisor == 1)
            return INT_MAX_32;
        if (divisor == INT_MIN_32)
            return 0;

        bool signFlag = false, dif = false;
        if (dividend < 0){
            if (dividend == INT_MIN_32){
                dividend = INT_MAX_32;
                dif = true;
            }
            else
                dividend *= (-1);
            signFlag = !signFlag;
        }
        if (divisor < 0){
            divisor *= (-1);
            signFlag = !signFlag;
        }
        
        int ret = 0;
        for (; ret <= INT_MAX_32 / divisor && divisor*ret <= dividend; ret++);

        ret--;
        if (dif){
            int difRet = INT_MAX_32 - divisor * ret + 1;
            if (difRet == divisor)
                ret++;
        }   
        if (signFlag)
            ret *= (-1);

        return ret;
        
    }
};


// To deal with extreme case is curbersome.
class Solution {
public:
    int divide(int dividend, int divisor) {
        if (dividend == INT_MIN && divisor == -1)
		    return INT_MAX;   
        if (dividend == INT_MIN && divisor == 1)
		    return INT_MIN;              
        
        int sign = (dividend > 0) ^ (divisor > 0);
        unsigned long dividendU = abs(dividend);  // Use unsinged for shift, use long for extreme limits.
        unsigned long divisorU = abs(divisor);
        int cnt = 0, res = 0;
        while (dividendU >= divisorU) {
            divisorU <<= 1;
            cnt++;
        }

        while (cnt > 0) {
            cnt--;
            divisorU >>= 1;
            if (dividendU >= divisorU) {
                res += (1 << cnt);
                dividendU -= divisorU;
            }
        }

        if (sign) 
            res *= -1;
        if (INT_MIN <= res && res <= INT_MAX)
            return res;
        return INT_MAX;
    }
};
