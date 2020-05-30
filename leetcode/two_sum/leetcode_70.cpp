class Solution {
public:
    int climbStairs(int n) {
        if (n == 1)
            return 1;
        if (n == 2)
            return 2;

        vector<int> movingWin(3, 0);
        movingWin[0] = 1;
        movingWin[1] = 2;
        
        // f(n) = f(n-1) + f(n-2)
        for(int i = 2; i < n; i++){
            movingWin[2] = movingWin[0] + movingWin[1];
            movingWin[0] = movingWin[1];
            movingWin[1] = movingWin[2];
        }
            return movingWin[2];
    }
};
