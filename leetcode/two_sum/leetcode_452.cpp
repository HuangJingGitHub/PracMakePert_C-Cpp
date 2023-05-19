class Solution {
public:
    int findMinArrowShots(vector<vector<int>>& points) {
        sort(points.begin(), points.end(), [](auto& a, auto& b) {
            return a.front() < b.front();
            });
        
        int res = 1, pre_x_end = points[0][1];
        for (int i = 1; i < points.size(); i++) {
            if (points[i][0] > pre_x_end) {
                res++;
                pre_x_end = points[i][1];
            }
            else {
                pre_x_end = min(pre_x_end, points[i][1]);
            }
        }
        return res;
    }
};
