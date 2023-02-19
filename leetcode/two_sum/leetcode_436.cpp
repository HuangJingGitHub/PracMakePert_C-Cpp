class Solution {
public: bool cmp(vector<int>& a, vector<int>& b) {
    return a[0] < b[0];
}
    static
    vector<int> findRightInterval(vector<vector<int>>& intervals) {
        vector<int> res(intervals.size(), -1);
        for (int i = 0; i < intervals.size(); i++) 
            intervals[i].push_back(i);
        
        sort(intervals.begin(), intervals.end());

        for (int i = 0; i < intervals.size(); i++) {
            int original_idx = intervals[i][2];

            int left = i, right = intervals.size() - 1, mid = 0;
            while (left < right) {
                mid = left + (right - left) / 2;
                if (intervals[mid][0] >= intervals[i][1])
                    right = mid;
                else
                    left = mid + 1;
            }
            if (intervals[right][0] >= intervals[i][1])
                res[original_idx] = intervals[right][2];
        }
        
        return res;
    }
};
