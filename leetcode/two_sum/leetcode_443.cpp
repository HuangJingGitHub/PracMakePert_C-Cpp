class Solution {
public:
    int compress(vector<char>& chars) {
        if (chars.size() == 1)
            return 1;

        int slowPt = 0, fastPt = 0, cnt = 1;
        while (fastPt < chars.size()) {
            cnt = 1;
            while (fastPt + cnt < chars.size() && chars[fastPt] == chars[fastPt + cnt])
                cnt++;
            chars[slowPt] = chars[fastPt];
            slowPt++;
            fastPt += cnt;
            if (cnt > 1) {
                string cntStr = to_string(cnt);
                for (char numChar : cntStr)
                    chars[slowPt++] = numChar;
            }
        }
        return slowPt;
    }
};
