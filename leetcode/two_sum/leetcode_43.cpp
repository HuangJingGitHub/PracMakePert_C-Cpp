// This is the personal solution which misunderstands the problem as the problem wants to solve the very important problem
// of multiplicaiton of very large numbers that cannot be performed by int muplicaiton in computer. That is to implement the
// vertical multiplication which can computate multiplication of really large numbers, like hundreds, thousands of digits as humans do.
class Solution {
public:
    string multiply(string num1, string num2) {
        long long int intNum1 = 0, intNum2 = 0, product;
        string ret;

        for (int i = num1.size() - 1; i >= 0; i--){
            char currentDigit = num1[i];
            int currentDigitInt = currentDigit - '0';
            intNum1 += currentDigitInt * pow(10, num1.size() - 1 - i);
        }

        for (int i = num2.size() - 1; i >= 0; i--){
            char currentDigit = num2[i];
            int currentDigitInt = currentDigit - '0';
            intNum2 += currentDigitInt * pow(10, num2.size() - 1 - i);
        }    
        product = intNum1 * intNum2;

        int digits = 1;
        for (; product - pow(10, digits) >= 0; digits++);
        if (product == pow(10, digits))
            digits++;
        for(long long int i = pow(10, digits-1); digits > 0; i = pow(10, digits-1)){
            char digitChar = product / i + '0';
            ret += digitChar;
            product -= i * (product / i);
            digits--;
        }    
       return ret; 
    }
};

// Very elegant solution. Refer to the offical discussion page.
class Solution {
public:
    string multiply(string num1, string num2) {
        int n1 = num1.size(), n2 = num2.size();
        string  res(n1 + n2, '0');

        for (int i = n2 - 1; i >= 0; i--) {
            for (int j = n1 - 1; j >= 0; j--) {
                int temp = (res[i + j + 1] - '0') + (num1[j] - '0') * (num2[i] - '0');
                res[i + j + 1] = temp % 10 + '0';
                res[i + j] += temp / 10;
            }
        }
        
        for (int i = 0; i < res.size(); i++) {
            if (res[i] != '0')
                return res.substr(i);
        }
        return "0";
    }
};
