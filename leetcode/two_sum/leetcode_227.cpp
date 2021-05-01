class Solution {
public:
    int calculate(string s) {
        int res = 0, num = 0, operant = 0, sign = 1;
        char operatorType = ' ';
        for (char c : s) {
            if (isdigit(c))
                num = num * 10 + (c - '0');
            else if (c == '+' || c == '-') {
                if (operatorType != ' ') {
                    if (operatorType == '*')
                        num *= operant;
                    else
                        num = operant / num;
                    operant = 0;
                    operatorType = ' ';
                }
                res += sign * num;
                num = 0;
                if (c == '+')
                    sign = 1;
                else
                    sign = -1;
            }
            else if (c == '*' || c == '/') {
                if (operatorType != ' ') {
                    if (operatorType == '*')
                        num *= operant;
                    else
                        num = operant / num;
                }
                operant = num;
                num = 0;
                operatorType = c;
            }
        }
        
        if (operatorType == ' ')
            res += sign * num;
        else if (operatorType == '*')
            res  += sign * (operant * num);
        else 
            res += sign * (operant / num);
        return res;
    }
};
