class Solution {
public:
    string fractionAddition(string expression) {
        int res_numerator = 0, res_denominator = 1;
        int start_idx = 0, end_idx = 1;

        while (end_idx < expression.size()) {
            while (end_idx < expression.size() 
                    && expression[end_idx] != '+' 
                    && expression[end_idx] != '-') 
                end_idx++;

            string cur_str = expression.substr(start_idx, end_idx - start_idx);
            cout << cur_str << "\n";

            int slash_idx = 0;
            while (cur_str[slash_idx] != '/')
                slash_idx++;
            int cur_numerator = stoi(cur_str.substr(0, slash_idx)),
                cur_denominator = stoi(cur_str.substr(slash_idx + 1)),
                gcd = GetGcd(abs(res_denominator), cur_denominator);
            int lcm = res_denominator * cur_denominator / gcd,
                res_denominator = lcm;
            
            int res_numerator = res_numerator * lcm / res_denominator + cur_numerator * lcm / cur_denominator;
            cout << res_numerator << "*" << lcm << "/" << res_denominator << "+" << cur_numerator << "*"
                 << lcm << "/" << cur_denominator << "=" << res_numerator << "\n"; 
            gcd = GetGcd(abs(res_numerator), res_denominator);
            res_numerator /= gcd;
            res_denominator /= gcd; 
            cout << "Res: " << res_numerator << "/" << res_denominator << '\n';   
            start_idx = end_idx;
            end_idx++;
        }
        return to_string(res_numerator) + "/" + to_string(res_denominator);
    }

    int GetGcd(int a, int b) {
        if (a == 0)
            return b;
            
        while (a != b) {
            if (a > b)
                a = a - b;
            else
                b = b - a;
        }
        return a;
    }
};
