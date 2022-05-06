class Solution {
public:
    int climbStairs(int n) {
        if (n == 1)
            return 1;
        if (n == 2)
            return 2;

        vector<int> movingWindow(3, 0);
        movingWindow[0] = 1;
        movingWindow[1] = 2;
        
        // f(n) = f(n-1) + f(n-2)
        for(int i = 2; i < n; i++) {
            movingWindow[2] = movingWindow[0] + movingWindow[1];
            movingWindow[0] = movingWindow[1];
            movingWindow[1] = movingWindow[2];
        }
            return movingWindow[2];
    }
};
