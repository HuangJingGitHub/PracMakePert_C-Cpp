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


class Solution {
public:
    int arrangeCoins(int n) {
        if (n == 1)
            return 1;
            
        long sum = 1, left = 1, right = n, mid;
        while (left < right) {
            mid = left + (right - left) / 2;
            sum = mid * (mid + 1) / 2;
            if (sum < n)
                left = mid + 1;
            else if (sum == n)
                return mid;
            else 
                right = mid;
        }

        return left - 1;
    }
};
