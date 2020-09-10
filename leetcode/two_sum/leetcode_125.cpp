class Solution {
public:
    bool isPalindrome(string s) {
        int i = 0, j = s.size() - 1;
        for (; i < j; i++, j--){
            char front = s[i], back = s[j];
            while (!( (front >= '0' && front <= '9') || (front >= 'a' && front <= 'z') || (front >= 'A' && front <= 'Z') )){
                i++;
                if (i < j)
                    front = s[i];
                else
                    break;
            }
            while (!( (back >= '0' && back <= '9') || (back >= 'a' && back <= 'z') || (back >= 'A' && back < 'Z') )){
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
