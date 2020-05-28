class Solution {
public:
    string addBinary(string a, string b) {
        int len_a = a.size(), len_b = b.size(), len_max = max(len_a, len_b);
        string res(len_max, '0');

        bool addOne = false;
        int addRes;
        for (int i = 1; i <= len_max; i++){
            if (addOne)
                res[len_max - i] = '1';
            addOne = false;

            if (i <= len_a && i <= len_b){
                addRes = a[len_a - i] + b[len_b - i];
                if (addRes == '1' + '1')
                    addOne = true;
                else 
                    addOne = false;
            }
            else if (i > len_a){
                addRes = b[len_b - i];
            }
            else
                addRes = a[len_a - i];
            
            if (addRes == '0' || addRes == '0' + '0' || addRes == '1' + '1'){
                if (res[len_max - i] == '0')
                    res[len_max - i] = '0';
                else
                    res[len_max - i] = '1';
            }
            else{
                if (res[len_max - i] == '0')
                    res[len_max - i] = '1';
                else{
                    res[len_max - i] = '0';
                    addOne = true;
                }
            }
        }

        if (addOne)
            res = "1" + res;
        return res;
    }
};
