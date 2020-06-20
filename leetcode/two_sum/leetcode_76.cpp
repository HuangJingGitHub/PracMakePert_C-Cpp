// good example of sliding window
class Solution {
public:
    unordered_map<char, int> ori, cnt;
    bool check()
    {
        for (const auto &p : ori)
            if (cnt[p.first] < p.second)
                return false;
        
        return true;
    }

    string minWindow(string s, string t) {
        for (const auto &c : t)
            ++ori[c];

        int l = 0, r = 0;
        int len = s.size() + 1, ansL = -1;
        
        // It is wired that s.size() conversion to int must be used here. Maybe because of the leetcode web
        while (r < int(s.size())){
            if (ori.find(s[r]) != ori.end())
                cnt[s[r]]++;
            r++;
            while (check() && l <= r){
                if (r - l + 1 < len){
                    len = r - l + 1;
                    ansL = l;
                }
                if (ori.find(s[l]) != ori.end())
                    cnt[s[l]]--;

                l++;
            }
        }

        return ansL == -1 ? "" : s.substr(ansL, len);
    }
};
