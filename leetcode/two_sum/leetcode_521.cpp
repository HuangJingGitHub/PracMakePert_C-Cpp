class Solution {
public:
    int findLUSlength(string a, string b) {
        if (a.size() != b.size())
            return max(a.size(), b.size());
        
        // find the first different letters from back to start
        for (int i = a.size() - 1; i >= 0; i--)
            if (a[i] != b[i])
                return i + 1;
        return -1;
    }
};
