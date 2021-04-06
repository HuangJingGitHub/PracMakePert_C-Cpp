class Solution {
public:
    string reverseOnlyLetters(string S) {
        int left = 0, right = S.size() - 1;
        while (left < right) {
            while (left < S.size() && !isalpha(S[left]))
                left++;
            while (right >= 0 && !isalpha(S[right]))
                right--;

            if (left < right) {
                swap(S[left], S[right]);
                left++;
                right--;
            }           
        }
        return S;
    }
};
