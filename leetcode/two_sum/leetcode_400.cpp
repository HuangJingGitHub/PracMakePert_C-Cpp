class Solution {
public:
    int findNthDigit(int n) {
        int idx = 0;
        long digitSum = 0;
        while (digitSum < n) {
            idx++;
            digitSum += (idx * 9 * pow(10, idx - 1));
        }

        int subIdx = n - (digitSum - idx * 9 * pow(10, idx - 1));
        int numPos = (subIdx - 1) / idx;
        int digitPos = (subIdx - 1) % idx;
        int targetNum = pow(10, idx - 1) + numPos;
        int res;

        for (int i = idx; i > digitPos; i--) {
            res = targetNum % 10;
            targetNum /= 10;
        }
        return res;
    }
};
