class Solution {
public:
    int calculate(string s) {
        stack<int> stk;
        int res = 0, num = 0, sign = 1;
        for (char c : s) {
            if (isdigit(c)) 
                num = num * 10 + (c - '0');
            
            else if (c == '+' || c == '-') {
                res += sign * num;
                num = 0;
                if (c == '+')
                    sign = 1;
                else
                    sign = -1;
            }
            else if (c == '(') {
                stk.push(res);
                stk.push(sign);
                num = 0;
                res = 0;
                sign = 1;
            }
            else if (c == ')') {
                res += sign * num;
                int preSign = stk.top();
                stk.pop(); 
                int preRes = stk.top();
                stk.pop();

                res = preRes + preSign * res;
                num = 0;
            }
        }
        res += sign * num;
        return res;
    }
};
