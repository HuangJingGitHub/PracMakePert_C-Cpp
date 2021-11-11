class Solution {
public:
    vector<char> vowels_{'a', 'e', 'i', 'o', 'u', 'A', 'E', 'I', 'O', 'U'};

    bool isVowel(char ch) {
        for (char& vowel : vowels_)
            if (ch == vowel)
                return true;
        return false;
    }

    string reverseVowels(string s) {
        int left = 0, right = s.size() - 1;
        char temp;

        while (left < right) {
            while (left < right && isVowel(s[left]) == false)
                left++;
            while (right > left && isVowel(s[right]) == false)
                right--;
            if (left == right)
                break;
            char temp = s[left];
            s[left] = s[right];
            s[right] = temp;
            left++;
            right--;
        }
        return s;
    }
};
