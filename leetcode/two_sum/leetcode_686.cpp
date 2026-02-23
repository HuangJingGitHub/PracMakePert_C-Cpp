class Solution {
public:
    int repeatedStringMatch(string a, string b) {
        string s = a;
        while (s.size() <= b.size())
            s += a;
        s += a;

        size_t found = s.find(b);
        if (found == string::npos)
            return -1;
        
        return (found + b.size() - 1) / a.size() + 1;
     }
};
