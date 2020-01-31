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
