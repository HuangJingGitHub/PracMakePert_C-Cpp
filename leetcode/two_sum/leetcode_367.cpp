class Solution {
public:
    bool isPerfectSquare(int num) {
        int square = 1, i = 1;
        while (square < num) {
            if (num - i * i < 2 * i + 1)
                return false;
            square += 2 * i + 1;
            i++;
        }
        return square == num;
    }
};
