class Solution {
public:
    string findLongestWord(string s, vector<string>& dictionary) {
        string res;

        for (string& str : dictionary) 
            if (canBeFormed(str, s)) {
                if (str.size() > res.size())
                    res = str;
                else if (str.size() == res.size() && lexicographicalCom(str, res))
                    res = str;
            }    
        return res;
    }

    bool canBeFormed(string& str, string& s) {
        if (str.size() > s.size())
            return false;
        
        int str_idx = 0, s_idx = 0;
        while (str_idx < str.size() && s_idx < s.size()) {
            if (str[str_idx] == s[s_idx]) {
                str_idx++;
                s_idx++;
            }
            else 
                s_idx++;
        }
        return str_idx == str.size();
    }

    bool lexicographicalCom(string& str_1, string& str_2) {
        int idx_1 = 0, idx_2 = 0;

        while (idx_1 < str_1.size() && idx_2 < str_2.size() 
                && str_1[idx_1] == str_2[idx_2]) {
                    idx_1++;
                    idx_2++;
        }

        if (idx_1 == str_1.size() && idx_2 < str_2.size())
            return true;
        else if (idx_1 < str_1.size() && idx_2 == str_2.size())
            return false;
        return str_1[idx_1] < str_2[idx_2];
    }
};
