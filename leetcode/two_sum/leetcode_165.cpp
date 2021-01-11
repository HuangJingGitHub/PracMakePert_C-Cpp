class Solution {
public:
    int compareVersion(string version1, string version2) {
        vector<string> strVec1, strVec2;
        for (int i = 0, j = 0; j < version1.size(); j++) {
            if (version1[j] == '.') {
                strVec1.push_back(version1.substr(i, j - i));
                i = j + 1;
            }
            if (j == version1.size() - 1)
                strVec1.push_back(version1.substr(i));
        }
        for (int i = 0, j = 0; j < version2.size(); j++) {
            if (version2[j] == '.') {
                strVec2.push_back(version2.substr(i, j - i));
                i = j + 1;
            }
            if (j == version2.size() - 1)
                strVec2.push_back(version2.substr(i));
        }


        int lenV1 = strVec1.size(), lenV2 = strVec2.size(), minLen = min(lenV1, lenV2), maxLen = max(lenV1, lenV2);
        for (int i = 0; i < minLen; i++) {
            int v1 = stoi(strVec1[i]), v2 = stoi(strVec2[i]);
            if (v1 < v2)
                return -1;
            else if (v1 > v2)
                return 1;
        }
        for (int i = minLen; i < maxLen; i++) {
            int v1 = (lenV1 < maxLen) ? 0 : stoi(strVec1[i]),
                v2 = (lenV2 < maxLen) ? 0 : stoi(strVec2[i]);
            if (v1 < v2)
                return -1;
            else if (v1 > v2)
                return 1;
        }
        return 0;
    }
};
