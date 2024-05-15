class Solution {
public:
    int nextGreaterElement(int n) {
        vector<int> digits;
        int original_num = n;
        while (original_num != 0) {
            digits.push_back(original_num % 10);
            original_num /= 10；
        }
        std::reverse(digits.begin(), digits.end());

        int digit_num = digits.size();
        vector<int> smaller_idx(digit_num, -1);
        return -1;
    }
};
