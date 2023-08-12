class Solution {
public:
    string smallestGoodBase(string n) {
        long long long_n = stol(n);
        long long res = long_n - 1;
        for (int s = 59; s >= 2; s--) {
            int k = pow(long_n, 1.0 / s);
            if (k > 1) {
                long long sum = 1, mul = 1;
                for (int i = 1; i <= s; i++) {
                    mul *= k;
                    sum += mul;
                }
                if (sum == long_n) {
                    res = k;
                    break;
                }
            }
        }

        return to_string(res);
    }
};
