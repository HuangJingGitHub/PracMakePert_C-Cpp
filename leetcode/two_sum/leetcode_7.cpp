class Solution {
public:
    int reverse(int x) {
        long num=0;
		while(x) {
			num = num * 10 + x % 10;
			x /= 10;
		}
		if(x < 0)
			num *= -1;
		if(num < -2147483648 || num > 2147483647)	
			num = 0;
		return num;
    }
};
