class Solution {
public:
    bool checkPerfectNumber(int num) {
        if (num == 1)
            return false;

        unordered_set<int> divisors{1};
        for (int i = 2; i <= sqrt(num); i++) {
            if (num % i == 0) {
                divisors.insert(i);
                divisors.insert(num / i);
            }
        }
        for (int div : divisors)
            num -= div;
        return num == 0;
    }
};

class Solution {
public:
    bool checkPerfectNumber(int num) {
        if (num == 1)
            return false;
        
        int sum = 1;
        for (int i = 2; i <= sqrt(num); i++) {
            if (num % i == 0) {
                sum += i;
                if (i != num / i)
                    sum += num / i;
            }
        }
        return sum == num;
    }
};
