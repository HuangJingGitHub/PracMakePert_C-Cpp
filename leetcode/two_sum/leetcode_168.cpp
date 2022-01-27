// Not correct for some cases
class Solution {
public:
    string convertToTitle(int n) {
        string res;
        int digitsNum = 1, powSum = 26, nCopy = n;
        while (n > powSum)
            powSum += pow(26, ++digitsNum);

        n -= (powSum - pow(26, digitsNum));
        vector<int> digits(digitsNum, 0);
        int idx = 0;
        while (idx < digitsNum) {
            digits[idx] = n / pow(26, digitsNum - 1 - idx);
            n = n % (int)pow(26, digitsNum - 1 - idx);
            idx++;
        }
        if (nCopy % 26 != 0 ) {
        for (int i = 0; i < digits.size() - 1; i++) 
            res += (char) (digits[i] + 65);
        res += (char) (digits.back() + 64);
        }
        else {
            res = string(digitsNum, 'Z');
            int idx = digits.size() - 1;
            while (digits[idx] == 0)
                idx--;
            res[idx] = (char)(digits[idx] + 64);
            for (; idx >= 0; idx--)
                res[idx] = (char)(digits[idx] + 64);
        }
        return res;
    }
};

// refer to the solution forum
class Solution {
public:
    string convertToTitle(int n) {
        string res;
        while (n > 0) {
            int c = n % 26;
            if (c == 0) {
                c = 26;
                n--;
            }
            res.insert(0, 1, (char) ('A' + c - 1));
            n /= 26;
        }
        return res;
    }
};


class Solution {
public:
    string convertToTitle(int n) {
       string res;

       while (n != 0) {
           n--;
           res.insert(0, 1, 'A' + n % 26);
           n /= 26;
       }
       return res;
    }
};
