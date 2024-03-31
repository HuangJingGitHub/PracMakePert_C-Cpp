class Solution {
public:
    string reverseStr(string s, int k) {
        int n = s.size();
        for (int left = 0; left < n; left += 2 * k) {
            int right = left + k - 1;
            right = min(right, n - 1);
            reverse(s, left, right);
        }

        return s;
    }

    void reverse(string& s, int left, int right) {
        while (left < right) {
            char temp = s[left];
            s[left] = s[right];
            s[right] = temp;
            left++;
            right--;
        }
    }
};
