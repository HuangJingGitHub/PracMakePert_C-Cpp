class Solution {
public:
    int largestPalindrome(int n) {
        if (n == 1)
            return 9;
        
        int max_n = pow(10, n) - 1;

        for (int firstPart = max_n; firstPart >= 0; firstPart--) {
            long palindrome = firstPart;
            for (int secondPart = firstPart; secondPart != 0; secondPart /= 10)
                palindrome = palindrome * 10 + secondPart % 10;

            for (long i = max_n; i * i >= palindrome; i--)
                if (palindrome % i == 0)
                    return (int) (palindrome % 1337);
        }
        return -1;
    }
};
