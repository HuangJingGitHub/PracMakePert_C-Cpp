class Solution {
public:
    int minTaps(int n, vector<int>& ranges) {
        int res = 0;
        vector<pair<int, int>> intervals;
        
        for (int i = 0; i < ranges.size(); i++)
            intervals.push_back(pair<int, int>(i - ranges[i], i + ranges[i]));
        sort(intervals.begin(), intervals.end());

        int max_right = 0, idx = 0;
        while (max_right < n) {
            int next_max_right = -1;
            while (idx < intervals.size() && intervals[idx].first <= max_right) {
                next_max_right = max(next_max_right, intervals[idx].second);
                idx++;
            }
            if (next_max_right == -1)
                return -1;
            max_right = next_max_right;
            res++;
        }
        return res;
    }
};
