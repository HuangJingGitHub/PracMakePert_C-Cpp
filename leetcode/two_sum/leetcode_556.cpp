class Solution {
public:
    int nextGreaterElement(int n) {
        vector<int> digits;
        int original_num = n;
        while (original_num != 0) {
            digits.push_back(original_num % 10);
            original_num /= 10;
        }
        std::reverse(digits.begin(), digits.end());

        int small_index = -1;
        for (int i = digits.size() - 2; i >= 0; i--) { 
            for (int j = i + 1; j < digits.size(); j++) {
                if (digits[j] > digits[i]) {
                    small_index = i;
                    break;
                }
            }
            if (small_index != -1)
                break;
        }
        if (small_index == -1)
            return -1;

        int min_larger_index = small_index + 1, larger_digit = 10;
        for (int i = min_larger_index; i < digits.size(); i++) {
            if (digits[i] > digits[small_index] && digits[i] < larger_digit) {
                larger_digit = digits[i];
                min_larger_index = i;
            }
        }

        std::swap(digits[small_index], digits[min_larger_index]);
        vector<int> reordered_digits(digits.size() - 1 - small_index, 0);
        for (int i = small_index + 1; i < digits.size(); i++)
            reordered_digits[i - small_index - 1] = digits[i];
        std::sort(reordered_digits.begin(), reordered_digits.end());
        for (int i = small_index + 1; i < digits.size(); i++)
            digits[i] = reordered_digits[i - small_index - 1]; 
        
        long long int res = digits[0];
        for (int i = 1; i < digits.size(); i++)
            res = (res * 10) + digits[i];
        
        if (res > INT_MAX)
            return -1;
        return res;
    }
};

// More concise
class Solution {
public:
    int nextGreaterElement(int n) {
        if (n < 10)
            return -1;
        
        vector<int> digits;
        while (n != 0) {
            digits.push_back(n % 10);
            n /= 10;
        }
        std::reverse(digits.begin(), digits.end());

        bool max_element = true;
        for (int i = digits.size() - 2; i >= 0; i--) {
            if (digits[i] < digits[i + 1]) {
                std::partial_sort(digits.begin() + i + 1, digits.end(), digits.end());
                for (int j = i + 1; j < digits.size(); j++) {
                    if (digits[j] > digits[i]) {
                        std::swap(digits[i], digits[j]);
                        max_element = false;
                        break;
                    }
                }
                break;
            }
        }
        if (max_element == true)
            return -1;

        long long res = 0;
        for (int& single_digit : digits)
            res = res * 10 + single_digit;
        return (res > INT_MAX) ? -1 : res;
    }
};
