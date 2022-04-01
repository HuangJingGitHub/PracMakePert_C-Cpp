class Solution {
public:
    int strStr(string haystack, string needle) {
        if (needle.size() == 0)
            return 0;
        if (haystack.size() < needle.size())
            return -1;

        for (int i = 0; i <= haystack.size() - needle.size(); i++){
            if (haystack[i] == needle[0]){
                string subStr = haystack.substr(i, needle.size());
                if (subStr == needle)
                    return i;
            }
        }
        return -1;
    }
};


// KMP
class Solution {
public:
    int strStr(string haystack, string needle) {
        if (needle.size() == 0)
            return 0;
        
        vector<int> next(needle.size(), 0);

        for (int left = 0, right = 1; right < needle.size(); right++) {
            while (left > 0 && needle[left] != needle[right])
                left = next[left - 1];
            if (needle[left] == needle[right])
                left++;
            next[right] = left;
        }
        for (int& num : next)
            cout << num << ", ";
        for (int i = 0, j = 0; i < haystack.size(); i++) {
            while (j > 0 && haystack[i] != needle[j])
                j = next[j - 1];
            if (haystack[i] == needle[j])
                j++;
            if (j == needle.size())
                return i - j + 1;
        }
        return -1;
    }
};


// refer to geeksforgeeks for KMP algorithm
class Solution {
public:
    int strStr(string haystack, string needle) {
        if (needle.size() == 0)
            return 0;
        
        vector<int> next(needle.size(), 0);

        for (int i = 1, len = 0; i < next.size(); ) {
            if (needle[len] == needle[i]) {
                len++;
                next[i] = len;
                i++;
            }
            else {
                if (len != 0) {
                    len = next[len - 1];
                }
                else {
                    next[i] = 0;
                    i++;
                }
            }
        }

        for (int i = 0, j = 0; i < haystack.size(); ) {
            if (haystack[i] == needle[j]) {
                i++;
                j++;
            }
            if (j == needle.size())
                return i - j;
            else if (haystack[i] != needle[j]) {
                if (j != 0)
                    j = next[j - 1];
                else
                    i++;
            }
        }
        return -1;
    }
};
