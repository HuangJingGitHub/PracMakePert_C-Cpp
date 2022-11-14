class Solution {
public:
    string removeOccurrences(string s, string part) {
        vector<int> lps(part.size(), 0);
        for (int i = 1, len = 0; i < part.size(); ) {
            if (part[len] == part[i]) {
                len++;
                lps[i] = len;
                i++;
            }
            else {
                if (len != 0)
                    len = lps[len - 1];
                else {
                    lps[i] = 0; 
                    i++;
                }
            }
        }

        int startIdx = strStr(s, part, lps);
        while (startIdx != -1) {
            s.erase(startIdx, part.size());
            startIdx = strStr(s, part, lps);
        }

        return s;
    }

    int strStr(string& s, string& needle, vector<int>& lps) {
        int i  = 0, j = 0;
        while(s.size() - i >= needle.size() - j) {
            if (j == needle.size())
                return i - j;

            if (s[i] == needle[j]) {
                i++;
                j++;
            }
            else {
                if (j > 0)
                    j = lps[j - 1];
                else
                    i++;
            }
        }
        return -1;
    }    
};


// built-in find method
class Solution {
public:
    string removeOccurrences(string s, string part) {
        size_t startIdx = s.find(part);
        while (startIdx != std::string::npos) {
            s.erase(startIdx, part.size());
            startIdx = s.find(part);
        }
        return s;
    } 
};
