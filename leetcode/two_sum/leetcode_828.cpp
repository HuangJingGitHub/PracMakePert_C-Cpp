class Solution {
public:
    int uniqueLetterString(string s) {
        long long res = 0;
        int len = s.size();
        vector<int> leftIdx(len, -1), rightIdx(len, len);
        map<char, int> idxDic;
        
        for (auto ch:s)
            idxDic[ch] = -1;
        for (int i = 0; i < len; i++){
            leftIdx[i] = idxDic[s[i]];
            idxDic[s[i]] = i;
        }
        
        for (auto ch:s)
            idxDic[ch] = len;
        for (int i = len-1; i >= 0; i--){
            rightIdx[i] = idxDic[s[i]];
            idxDic[s[i]] = i;
        }

        for (int i = 0; i < len; i++){
            res += (i - leftIdx[i])*(rightIdx[i] - i - 1);
        }

        return res % 1000000007;

    }
};
