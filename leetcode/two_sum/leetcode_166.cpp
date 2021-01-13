class Solution {
public:
    string fractionToDecimal(int numerator, int denominator) {
        vector<long long int> numeratorVec, resNum;
        string res;
        bool recurring = false, negative = false;
        int recurStart = 0;

        if (numerator % denominator == 0)
            return to_string(numerator / denominator);
        
        if ((numerator > 0 && denominator < 0) || (numerator < 0 && denominator > 0)) {
            negative = true;
        }
        numerator = abs(numerator);
        denominator = abs(denominator);
        
        numeratorVec.push_back(numerator);
        while (numerator % denominator != 0) {
            resNum.push_back(numerator / denominator);
            long long int newNumerator = 10 * (numerator % denominator);
            
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
            numerator = newNumerator;
        } 

        if (!recurring) {
            resNum.push_back(numerator / denominator);
            if (negative)
                res = "-" + to_string(resNum[0]) + ".";
            else
                res = to_string(resNum[0]) + ".";
            for (int i = 1; i < resNum.size(); i++)
                res += to_string(resNum[i]);
            return res;
        }
        else {
            if (negative)
                res = "-" + to_string(resNum[0]) + ".";
            else
                res = to_string(resNum[0]) + ".";
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
