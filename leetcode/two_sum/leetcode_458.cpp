class Solution {
public:
    int poorPigs(int buckets, int minutesToDie, int minutesToTest) {
        int res = 0;
        int maxRound = minutesToTest / minutesToDie + 1;
        while (pow(maxRound, res) < buckets)
            res++;
        return res;
    }
};
