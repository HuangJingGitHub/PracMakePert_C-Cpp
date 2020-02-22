class Solution {
public:
    int mySqrt(int x) {
        if (x == 0)
            return 0;
        
        int left = 1, right = x / 2, mid, sqr;

        while (left < right){
            mid = (left + right + 1) / 2;
            sqr = mid * mid;

            if (sqr > x)
                right = mid - 1;
            else
                left = mid;
        }
        return left;
    }
};
