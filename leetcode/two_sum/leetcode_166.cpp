// Implement manual devision process
class Solution {
public:
    string fractionToDecimal(int numerator, int denominator) {
        long long int num = numerator, dem = denominator;
        vector<long long int> numeratorVec, resNum;
        string res;
        bool recurring = false;
        int recurStart = 0;

        if (num % dem == 0)
            return to_string(num / dem);
        
        if ((num > 0 && dem < 0) || (num < 0 && dem > 0))
            res = "-";
        num = abs(num);
        dem = abs(dem);
        
        numeratorVec.push_back(num);
        while (num % dem != 0) {
            resNum.push_back(num / dem);
            long long int newNumerator = 10 * (num % dem);  // Avoid exceeding int range, e.g. 10 * (1 % 2147483648) > INT_MAX, use long long int
            
            for (int i = 0; i < numeratorVec.size(); i++) {
                if (numeratorVec[i] == newNumerator) {
                    recurring = true;
                    recurStart = i;
                    break;
                }
            }
            if (recurring)
                break;
            
            numeratorVec.push_back(newNumerator);
            num = newNumerator;
        } 

        if (!recurring) {
            resNum.push_back(num / dem);
            res += to_string(resNum[0]) + ".";
            for (int i = 1; i < resNum.size(); i++)
                res += to_string(resNum[i]);
            return res;
        }
        else {
            res += to_string(resNum[0]) + ".";
            int i = 1;
            while (i < recurStart) {
                res += to_string(resNum[i]);
                i++;
            }
            res += "(";
            while (i < resNum.size()) {
                res += to_string(resNum[i]);
                i++;
            }
            res += ")";
            return res;
        }
    }
};
