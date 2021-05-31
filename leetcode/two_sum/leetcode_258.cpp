class Solution {
public:
    int addDigits(int num) {
        int digit_sum = num;
        while (digit_sum > 9) {
            digit_sum = 0;
            while (num != 0) {
                digit_sum += num % 10;
                num /= 10;
            }
            num = digit_sum;
        }
        return digit_sum;
    }
};


// refer to the discussion for math derivaiton
class Solution {
public:
    int addDigits(int num) {
        if (num <= 9)
            return num;        
        if (num % 9 == 0)
            return 9;
        return num % 9;
    }
};
