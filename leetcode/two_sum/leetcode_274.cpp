class Solution {
public:
    int hIndex(vector<int>& citations) {
        int res = citations[0] > 0 ? 1 : 0;
        sort(citations.begin(), citations.end());
        for (int i = 1; i < citations.size(); i++) {
            if (citations[i] > res && citations[i - res] >= res + 1)
                res++;
        }
        return res;
    }
};
