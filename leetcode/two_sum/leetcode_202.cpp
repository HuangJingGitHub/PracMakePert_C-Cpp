class Solution {
public:
    bool isHappy(int n) {
        unordered_set<int> visited;
        int squareSum = 0;

        while (true) {
            while (n != 0) {
                int digit = n % 10;
                squareSum += digit * digit;
                n /= 10;
            }
            if (squareSum == 1)
                return true;
            else if (visited.find(squareSum) != visited.end())
                return false;
            else {
                visited.insert(squareSum);
                n = squareSum;
                squareSum = 0;
            }
        }
    }
};
