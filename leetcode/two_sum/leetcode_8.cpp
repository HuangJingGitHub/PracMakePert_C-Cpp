class Solution {
public:
    int myAtoi(string s) {
        int len = s.size(), idx = 0, res = 0, sign = 1;
        
        while (idx < len && s[idx] == ' ')
            idx++;
        if (idx == len)
            return 0;
        if (s[idx] == '+')
            idx++;
        else if (s[idx] == '-') {
            sign = -1;
            idx++;
        }

        while (idx < len) {
            char curChar = s[idx];
            if (curChar < '0' || curChar > '9')
                break;
            if (res > INT_MAX / 10 || (res == INT_MAX / 10 && curChar - '0' > INT_MAX % 10))
                return INT_MAX;
            else if (res < INT_MIN / 10 || (res == INT_MIN / 10 && curChar - '0' > -(INT_MIN % 10)))
                return INT_MIN;
            res = res * 10 + sign * (curChar - '0');
            idx++;
        }
        return res;
    }
};
