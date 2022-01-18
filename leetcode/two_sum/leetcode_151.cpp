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


class Solution {
public:
    string reverseWords(string s) {
        string res;
        int idx2 = s.size() - 1, idx1 = 0;

        while (idx1 >= 0) {
            while (idx2 >= 0 && s[idx2] == ' ')
                idx2--;
            if (idx2 < 0)
                break;

            idx1 = idx2 - 1;
            while (idx1 >= 0 && s[idx1] != ' ')
                idx1--;
            
            res += (" " + s.substr(idx1 + 1, idx2 - idx1));
            idx2 = idx1;
        }

        res = res.substr(1);
        return res;
    }
};
