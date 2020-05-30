// The devision will lead to overflow as the number can be quite large.
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

// Newton method considering the quadratic function f(cur) = cur^2 - x
class Solution {
public:
    int mySqrt(int x) {
        if (x == 0)
            return 0;
    double cur = 1.0, pre;

    while (true){
        pre = cur;
        cur = cur / 2 + x / (2 * cur);
        if (abs(pre - cur) < 0.1)
            return int(cur);
    }
    }
};
