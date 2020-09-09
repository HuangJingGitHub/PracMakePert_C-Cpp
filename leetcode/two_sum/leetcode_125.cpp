class Solution {
public:
    bool isPalindrome(string s) {
        int i = 0, j = s.size() - 1;
        for (; i < j; i++, j--){
            char front = s[i], back = s[j];
            while (!( (front > 47 && front < 58) || (front > 64 && front < 91) || (front > 96 && front < 123) )){
                i++;
                if (i < j)
                    front = s[i];
                else
                    break;
            }
            while (!( (back > 47 && back < 58) || (back > 64 && back < 91) || (back > 96 && back < 123) )){
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
