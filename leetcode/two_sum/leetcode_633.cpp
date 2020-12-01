class Solution {
public:
    bool judgeSquareSum(int c) {
        long int j = sqrt(c), i = 0, squareSum;
        while (i <= j){
            squareSum = i * i + j * j;
            if (squareSum > c)
                j--;
            else if (squareSum < c)
                i++;
            else
                return true;
        }
        return false;
    }
};
