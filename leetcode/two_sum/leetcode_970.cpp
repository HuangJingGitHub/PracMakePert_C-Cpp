class Solution {
public:
    vector<int> powerfulIntegers(int x, int y, int bound) {
        vector<int> res;
        set<int> log;

        for (int a = 1; a <= bound; a *= x) {
            for (int b = 1; a + b <= bound; b *= y) {
                if (log.find(a + b) == log.end()) {
                    res.push_back(a + b);
                    log.insert(a + b);
                }
                if (y == 1)
                    break;
            }
            if (x == 1)
                break;
        }
        return res;
    }
};
