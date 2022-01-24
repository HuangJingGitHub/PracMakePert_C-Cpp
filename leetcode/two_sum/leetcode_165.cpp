class Solution {
public:
    int compareVersion(string version1, string version2) {
        vector<int> idxVec1{-1}, idxVec2{-1};
        for (int i = 0; i < version1.size(); i++) {
            if (version1[i] == '.') 
                idxVec1.push_back(i);
        }
        idxVec1.push_back(version1.size());

        for (int i = 0; i < version2.size(); i++) {
            if (version2[i] == '.')
                idxVec2.push_back(i);
        }
        idxVec2.push_back(version2.size());

        int lenV1 = idxVec1.size() - 1, lenV2 = idxVec2.size() - 1, minLen = min(lenV1, lenV2), maxLen = max(lenV1, lenV2);
        for (int i = 0; i < minLen; i++) {
            int v1 = stoi(version1.substr(idxVec1[i] + 1, idxVec1[i + 1] - idxVec1[i] - 1)), 
                v2 = stoi(version2.substr(idxVec2[i] + 1, idxVec2[i + 1] - idxVec2[i] - 1));
            if (v1 < v2)
                return -1;
            else if (v1 > v2)
                return 1;
        }
        
        for (int i = minLen; i < maxLen; i++) {
            int v1 = (lenV1 < maxLen) ? 0 : stoi(version1.substr(idxVec1[i] + 1, idxVec1[i + 1] - idxVec1[i] - 1)),
                v2 = (lenV2 < maxLen) ? 0 : stoi(version2.substr(idxVec2[i] + 1, idxVec2[i + 1] - idxVec2[i] - 1));
            if (v1 < v2)
                return -1;
            else if (v1 > v2)
                return 1;
        }
        return 0;
    }
};


class Solution {
public:
    int compareVersion(string version1, string version2) {
        vector<int> dotIdx1, dotIdx2;
        
        for (int i = 0; i < version1.size(); i++)
            if (version1[i] == '.')
                dotIdx1.push_back(i);
        dotIdx1.push_back(version1.size());
        for (int i = 0; i < version2.size(); i++)
            if (version2[i] == '.')
                dotIdx2.push_back(i);
        dotIdx2.push_back(version2.size());
        
        vector<int> versionVec1, versionVec2;
        versionVec1.push_back(stoi(version1.substr(0, dotIdx1[0])));
        for (int i = 0; i < dotIdx1.size() - 1; i++)
            versionVec1.push_back(stoi(version1.substr(dotIdx1[i] + 1, dotIdx1[i + 1] - dotIdx1[i])));

        versionVec2.push_back(stoi(version2.substr(0, dotIdx2[0])));
        for (int i = 0; i < dotIdx2.size() - 1; i++)
            versionVec2.push_back(stoi(version2.substr(dotIdx2[i] + 1, dotIdx2[i + 1] - dotIdx2[i])));
            
        if (versionVec1.size() > versionVec2.size()) {
            for (int i = versionVec2.size(); i < versionVec1.size(); i++)
                versionVec2.push_back(0);
        }
        else {
            for (int i = versionVec1.size(); i < versionVec2.size(); i++)
                versionVec1.push_back(0);
        }

        for (int i = 0; i < versionVec1.size(); i++)
            if (versionVec1[i] > versionVec2[i])
                return 1;
            else if (versionVec1[i] < versionVec2[i])
                return -1;
        return 0;
    }
};
