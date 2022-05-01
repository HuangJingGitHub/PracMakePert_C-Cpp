// Personal solution, doesn't work. Only work for sorted intervals. Not general.
class Solution {
public:
    vector<vector<int>> merge(vector<vector<int>>& intervals) {
        int n = intervals.size();
        vector<vector<int>> res;
        if (n == 0 || n == 1)
            return intervals;
        vector<int> curIntval = intervals[0];

        for(int i = 0; i < n - 1; i++){
            if (curIntval[1] < intervals[i+1][0] || curIntval[0] > intervals[i+1][1]){
                res.push_back(curIntval);
                curIntval = intervals[i+1];
                continue;
            }
            else{ 
                curIntval[0] = min(curIntval[0], intervals[i+1][0]);
                curIntval[1] = max(curIntval[1], intervals[i+1][1]);
            }
        }
        res.push_back(curIntval);

        for(int i = res.size() - 1; i > 0; i--){
            if (curIntval[1] < res[i-1][0] || curIntval[0] > res[i-1][1]){
                curIntval = res[i-1];
                continue;
            }
            else{ 
                curIntval[0] = min(curIntval[0], res[i-1][0]);
                curIntval[1] = max(curIntval[1], res[i-1][1]);
                res.pop_back();
                res.pop_back();
                res.push_back(curIntval);
            }
        }
        return res;
    }
};

// good
class Solution {
public:
    vector<vector<int>> merge(vector<vector<int>>& intervals) {
        int n = intervals.size();
        vector<vector<int>> res;
        vector<int> start, end;

        for (int i = 0; i < n; i++) {
            start.push_back(intervals[i][0]);
            end.push_back(intervals[i][1]);
        }
        sort(start.begin(), start.end());
        sort(end.begin(), end.end());

        for (int i = 0, j = 0; i < n; i++) {
            if (i == n - 1 || start[i+1] > end[i]){
                res.push_back({start[j], end[i]});
                j = i + 1;
            }
        }
        return res;
    }
};
