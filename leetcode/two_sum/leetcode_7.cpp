class Solution {
public:
    int reverse(int x) {
        long num=0;
	while (x) {
		num = num * 10 + x % 10;
		x /= 10;
	}
	if (x < 0)
		num *= -1;
	if (num < -2147483648 || num > 2147483647)	
		num = 0;
	return num;
    }
};

class Solution {
public:
    int reverse(int x) {
        int res = 0;
	while (x != 0) {
		int digit = x % 10;
		if (res > INT_MAX / 10 || (res == INT_MAX / 10 && digit > 7)
		    return 0;
		if (res < INT_MIN / 10 || (res == INT_MIN / 10 && digit < -8)
		    return 0;
		res = res * 10 + digit;
	}
	return res;
    }
};

		    class Solution {
public:
    int reverse(int x) {
        int sign = x >= 0 ? 1 : -1, res = 0;
        if (x == INT_MIN)
            return 0;
        
        x = abs(x);
        while (x != 0) {
            if (res > INT_MAX / 10 || (res == INT_MAX / 10 && x % 10 > INT_MAX % 10))
                return 0;

            res = res * 10 + x % 10;
            x /= 10;
        }

        return sign * res;
    }
};
