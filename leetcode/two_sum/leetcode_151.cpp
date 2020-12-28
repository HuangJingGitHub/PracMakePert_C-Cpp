class Solution {
public:
    string reverseWords(string s) {
        string res;
        for (int i = s.size() - 1; i >= 0; i--) {
            if (s[i] == ' ')
                continue;
            int j = i - 1;
            while (j >= 0 && s[j] != ' ') 
                j--;
            res += s.substr(j + 1, i - j);
            res += " ";
            i = j;
        }
        res.pop_back();
        return res;
    }
};
