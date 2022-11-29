class Solution {
public:
    string toHex(int num) {
        long int complement = pow(2, 32), long_num = num;
        if (num < 0)
            long_num += complement;

        vector<char> digit{'a', 'b', 'c', 'd', 'e', 'f'};
        string res;
        while (long_num / 16 >= 0) {
            int mod = long_num % 16;
            if (mod <= 9)
                res += to_string(mod);
            else
                res += digit[mod - 10];
            long_num /= 16;

            if (long_num == 0)
                break;
        }
        reverse(res.begin(), res.end());
        
        return res;
    }
};
