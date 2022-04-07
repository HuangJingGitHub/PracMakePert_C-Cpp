class Solution {
public:
    bool isPalindrome(int x) {
        string strX = to_string(x);

        int left = 0, right = strX.size() - 1;
        while (left < right) {
            if (strX[left] != strX[right])
                return false;
            left++;
            right--;
        }
        return true;
    }
};
