class Solution {
public:
    string convertToBase7(int num) {
        if (num == 0)
            return "0";
            
        string res;
        int abs_num = abs(num);
        vector<int> remainder;

        while (abs_num != 0) {
            remainder.push_back(abs_num % 7);
            abs_num /= 7;
        }

        if (num < 0)
            res = "-";
        for (int i = remainder.size() - 1; i >= 0; i--)
            res += to_string(remainder[i]);

        return res;
    }
};
