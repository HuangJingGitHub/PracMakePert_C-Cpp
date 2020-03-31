class Solution {
public:
    vector<int> plusOne(vector<int>& digits) {
        if (digits.back() < 9){
            digits.back() += 1;
            return digits;
        }
        else{
            int i = digits.size();
            for (; digits[i] == 9 && i >= 0; i--){
                digits[i] = 0;
            }
            if (i>-1){
                digits[i] += 1;
            }
            else 
                digits.insert(digits.begin(), 1);
        }
        return digits;
        
    }
};
