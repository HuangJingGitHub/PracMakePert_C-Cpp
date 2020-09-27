class Solution {
public:
    int countTriplets(vector<int>& A) {
        unordered_map<int, int> triple;
        int res = 0;

        for (auto item1:A)
            for (auto item2:A)
                triple[item1 & item2]++;
        for (auto item1:triple)
            for (auto item2:A)
                if ((item1.first & item2) == 0)
                    res += item1.second;

        return res;
    }
};
