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
