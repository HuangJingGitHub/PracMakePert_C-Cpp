// Not compact via personal understanding, but actually it is just a transformation.
class Solution {
public:
    int titleToNumber(string s) {
        int res = 0;
        for (int i = 1; i < s.size(); i++)
            res += pow(26, i);
        res += (int)(s.back() - 'A' + 1);

        for (int i = s.size() - 2; i >= 0; i--) {
            int power = s.size() - i - 1;
            res += pow(26, power) * (int)(s[i] - 'A');
        }
        return res;
    }
};

// compact sol 1
class Solution {
public:
    int titleToNumber(string s) {
        int res = 0;
        for (int i = 0; i < s.size(); i++) {
            int digit = s[i] - 'A' + 1;
            res = 26 * res + digit;
        }
        return res;
    }
};

// Compact sol 2
class Solution {
public:
    int titleToNumber(string s) {
        int res = 0;
        for (int i = s.size() - 1; i >= 0; i--) {
            int digit = s[i] - 'A' + 1;
            res += digit * pow(26, s.size() - i - 1);
        }
        return res;
    }
};

