class Solution {
public:
    string complexNumberMultiply(string num1, string num2) {
        int plus_sign_idx_1 = 0, plus_sign_idx_2 = 0;
        while (num1[plus_sign_idx_1] != '+')
            plus_sign_idx_1++;
        while (num2[plus_sign_idx_2] != '+')
            plus_sign_idx_2++;
        
        int real_1 = stoi(num1.substr(0, plus_sign_idx_1 + 1)),
            imaginary_1 = stoi(num1.substr(plus_sign_idx_1 + 1, num1.size() - plus_sign_idx_1)),
            real_2 = stoi(num2.substr(0, plus_sign_idx_2 + 1)),
            imaginary_2 = stoi(num2.substr(plus_sign_idx_2 + 1, num2.size() - plus_sign_idx_2));
        int res_real = real_1 * real_2 - imaginary_1 * imaginary_2,
            res_imaginary = real_1 * imaginary_2 + real_2 * imaginary_1;
        return to_string(res_real) + "+" + to_string(res_imaginary) + "i";
    }
};
