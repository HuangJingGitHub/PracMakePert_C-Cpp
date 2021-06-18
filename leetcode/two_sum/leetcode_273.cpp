class Solution {
public:
        vector<string> low = {"", "One", "Two", "Three", "Four", "Five", "Six", "Seven", "Eight", "Nine"};
        vector<string> mid  = {"Ten","Eleven","Twelve","Thirteen","Fourteen","Fifteen","Sixteen","Seventeen","Eighteen","Nineteen"};
        vector<string> high = {"","","Twenty","Thirty","Forty","Fifty","Sixty","Seventy","Eighty","Ninety"};
    
    string numberToWords(int num) {
        if (num == 0)
            return "Zero";
        
        int part1 = num % 1000;
        num /= 1000;
        int part2 = num % 1000;
        num /= 1000;
        int part3 = num % 1000;
        num /= 1000;
        int part4 = num;

        string res = "";
        if (part4 != 0)
            res = buildNumber(part4) + "Billion ";
        if (part3 != 0)
            res = res + buildNumber(part3) + "Million ";
        if (part2 != 0)
            res = res + buildNumber(part2) + "Thousand ";
        if (part1 != 0)
            res = res + buildNumber(part1);
        
        trim(res);
        return res;
    }

    string buildNumber(int num) {
        int a = num % 10;
        num /= 10;
        int b = num % 10;
        num /= 10;
        int c = num;

        string res = "";

        if (c != 0)
            res = low[c] + " " + "Hundred ";
        if (b == 1)
            res = res + mid[a];
        else if (b == 0)
            res = res + low[a];
        else
            res = res + high[b] + " " + low[a];
        
        trim(res);
        return res + " ";
    }

    void trim(string& str) {
        int end_idx = str.size() - 1;
        for (; end_idx >= 0; end_idx--)
            if (str[end_idx] != ' ')
                break;
        str = str.substr(0, end_idx + 1);
    }
};
