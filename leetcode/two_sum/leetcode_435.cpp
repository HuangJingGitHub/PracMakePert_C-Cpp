class Solution {
public:
    static bool cmp(const vector<int>& a, vector<int>& b) {
        return a[0] < b[0];
    }

    int eraseOverlapIntervals(vector<vector<int>>& intervals) {
        sort(intervals.begin(), intervals.end(), cmp);
        
        int res = 0;
        int end = intervals[0][1];
        for (int i = 1; i < intervals.size(); i++) {
            if (intervals[i][0] >= end)
                end = intervals[i][1];
            else {
                end = min(end, intervals[i][1]);
                res++;
            }
        }    
        return res;
    }
};
