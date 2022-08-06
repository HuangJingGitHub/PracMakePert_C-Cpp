class Solution {
public:
    bool isPalindrome(string s) {
        int i = 0, j = s.size() - 1;
        for (; i < j; i++, j--){
            char front = s[i], back = s[j];
            while (!isalpnum(front)){
                i++;
                if (i < j)
                    front = s[i];
                else
                    break;
            }
            while (!isalnum(front)){
                j--;
                if (j > i)
                    back = s[j];
                else
                    break;
            } 

            if (front == back || (abs(front - back) == 32 && max(front, back) > 96 && max(front, back) < 123))
                continue;
            else
                break;
        }
        return i >= j;
    }
};


class Solution {
public:
    bool isPalindrome(string s) {
        int left = 0, right = s.size() - 1;
        while (left < right) {
            while (left < s.size() && !isalnum(s[left]))
                left++;
            while (right >= 0 && !isalnum(s[right]))
                right--;
            if (left == s.size() || right < 0)
                break;

            if (s[left] == s[right] || (max(s[left], s[right]) - min(s[left], s[right]) == 32 && max(s[left], s[right]) >= 'a')) {
                left++;
                right--;
                continue;
            }
            else
                break;
        }
        return left >= right;
    }
};
