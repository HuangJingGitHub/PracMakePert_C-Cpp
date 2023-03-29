class Solution {
public:
    int numberOfBoomerangs(vector<vector<int>>& points) {
        int point_num = points.size(), res = 0;
        vector<unordered_map<int, int>> distanceLog(point_num);
        for (int i = 0; i < point_num; i++)
            for (int j = i + 1; j < point_num; j++) {
                int diffX = points[i][0] - points[j][0],
                    diffY = points[i][1] - points[j][1],
                    distanceSquare = diffX * diffX + diffY * diffY;
                distanceLog[i][distanceSquare]++;
                distanceLog[j][distanceSquare]++;
            }
        for (int i = 0; i < point_num; i++) 
            for (auto it = distanceLog[i].begin(); it != distanceLog[i].end(); it++) {
                if (it->second >= 2)
                    res += it->second * (it->second - 1);
            }
        return res;
    }
};
