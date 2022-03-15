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


class Solution {
public:
    bool isPerfectSquare(int num) {
        int i = 1;
        while (i * i< num) {
            if (num - i * i < 2 * i + 1)
                return false;
            i++;
        }
        return i * i == num;
    }
};

class Solution {
public:
    bool isPerfectSquare(int num) {
        int low = 1, high = num;

        while (low <= high) {
            int mid = low + (high - low) / 2;
            int quotient = num / mid;
            if (quotient == mid) {
                if (num % quotient == 0)
                    return true;
                high--;
            }
            else if (quotient < mid)
                high = mid - 1;
            else
                low = mid + 1;
        }
        return false;
    }
};
