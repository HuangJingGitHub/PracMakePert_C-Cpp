// mathematical solution
class Solution {
public:
    int integerBreak(int n) {
        if (n <= 3)
            return n - 1;
        
        int a = n / 3, b = n % 3;
        if (b == 0)
            return pow(3, a);
        if (b == 1)
            return pow(3, a - 1) * 4;
        return pow(3, a) * 2;

    }
};


// dp solution
class Solution {
public:
    int integerBreak(int n) {
        vector<int> dp(n + 1, 1);

        // pay attention j * (i - j) is a new possible break pair with i - j complete, rather than broken
        for (int i = 2; i <= n; i++)
            for (int j = 1; j <= i - 1; j++)
                dp[i] = max({dp[i], j * dp[i - j], j * (i - j)});  
        return dp.back();
    }
};
