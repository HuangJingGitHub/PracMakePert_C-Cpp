class Solution {
public:
    string addStrings(string num1, string num2) {
        int carry = 0, len1 = num1.size(), len2 = num2.size(), len = max(len1, len2), curdigit;
        string res;
        if (len1 > len2)
            num2 = string(len1 - len2, '0') + num2;
        else if (len1 < len2)
            num1 = string(len2 - len1, '0') + num1;
        
        for (int i = len - 1; i >= 0; i--){
            curdigit = num1[i] - '0' + num2[i] - '0' + carry;
            res += to_string(curdigit % 10);
            carry = curdigit / 10;
        }
        if (carry == 1)
            res += "1";

        reverse(res.begin(), res.end());
        return res;
    }
};
