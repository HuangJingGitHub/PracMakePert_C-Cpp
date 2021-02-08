class Solution {
public:
    bool isIsomorphic(string s, string t) {
        unordered_map<char, char> forwardMap, backMap;
        for (int i = 0; i < s.size(); i++) {
            if (forwardMap.find(s[i]) == forwardMap.end()){
                if (backMap.find(t[i]) != backMap.end())
                    return false;
                else {
                    forwardMap[s[i]] = t[i];
                    backMap[t[i]] = s[i];
                }
            }
            else if (forwardMap[s[i]] != t[i])
                return false;
        }
        return true;
    }
};
