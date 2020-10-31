// in-place, add to the end
class Solution {
public:
    string replaceSpaces(string S, int length) {
        int j = S.size() - 1;
        for (int i = length - 1; i >= 0; i--){
            if (S[i] == ' '){
                S[j--] = '0';
                S[j--] = '2';
                S[j--] = '%';
            }
            else
                S[j--] = S[i];
        }
        return S.substr(j + 1);
    }
};

// new string
class Solution {
public:
    string replaceSpaces(string S, int length) {
        string res;
        for (int i = 0; i < length; i++){
            if (S[i] == ' ')
                res += "%20";
            else
                res += S[i];
        }
        return res;
    }
};
