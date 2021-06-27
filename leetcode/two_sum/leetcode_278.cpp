// The API isBadVersion is defined for you.
// bool isBadVersion(int version);

class Solution {
public:
    int firstBadVersion(int n) {
        int left = 1, right = n, mid;
        while (left <= right) {
            mid = left + (right - left) / 2;
            if (isBadVersion(mid) == true) {
                if (mid == 1 || isBadVersion(mid - 1) == false)
                    return mid;
                right = mid - 1;
            }
            else
                left = mid + 1;
        }
        return mid;
    }
};
