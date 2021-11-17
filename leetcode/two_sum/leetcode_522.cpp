class Solution {
public:
    struct comparer {
        bool operator() (const string& a, const string& b) {
            return a.size() > b.size();
        }
    };

    int findLUSlength(vector<string>& strs) {
        comparer c;
        sort(strs.begin(), strs.end(), c);

        for (int i = 0; i < strs.size(); i++) {
            int j = 0;
            for (; j < strs.size(); j++) {
                if (i == j)
                    continue;
                else if (strs[i].size() > strs[j].size())
                    return strs[i].size();
                else if (isSubsequence(strs[i], strs[j]))
                    break;
            }
            if (j == strs.size())
                return strs[i].size();
        }
        return -1;
    }
    
    bool isSubsequence(const string& s1, const string& s2) {
        int idx1 = 0, idx2 = 0;
        while (idx1 < s1.size() && idx2 < s2.size()) {
            if (s1[idx1] == s2[idx2]) {
                idx1++;
                idx2++;
            }
            else
                idx2++;
        }

        // see whether s1 is a subsequence of s2
        return idx1 == s1.size();
    }
};
