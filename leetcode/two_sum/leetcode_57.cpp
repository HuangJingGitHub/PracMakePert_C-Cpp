// Can see the favoriate solution for this. Easy to understand.
class Solution {
public:
    vector<vector<int>> insert(vector<vector<int>>& intervals, vector<int>& newInterval) {
        vector<vector<int>> res;
        int i;
        for (i = 0; i < intervals.size(); i++){
            if (newInterval[0] > intervals[i][1])
                res.push_back(intervals[i]);
            else
                break;
        }
        res.push_back(newInterval);

        for (; i < intervals.size(); i++){
            if (res.back()[1] < intervals[i][0])
                res.push_back(intervals[i]);
            else{
                res.back()[0] = min(res.back()[0], intervals[i][0]);
                res.back()[1] = max(res.back()[1], intervals[i][1]);
            }
        }

        return res;
    }
};
