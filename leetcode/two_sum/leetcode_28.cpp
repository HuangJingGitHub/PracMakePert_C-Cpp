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
