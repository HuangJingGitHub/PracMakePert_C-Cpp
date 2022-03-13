class Solution {
public:
    bool isRectangleCover(vector<vector<int>>& rectangles) {
        int bottomLeftX = INT_MAX,
            bottomLeftY = INT_MAX,
            topRightX = INT_MIN,
            topRightY = INT_MIN;
        long int sumArea = 0;
        unordered_set<string> vertexSet;

        for (auto& rec : rectangles) {
            bottomLeftX= min(bottomLeftX, rec[0]);
            bottomLeftY = min(bottomLeftY, rec[1]);
            topRightX = max(topRightX, rec[2]);
            topRightY = max(topRightY, rec[3]);

            vector<string> vertexStr{to_string(rec[0]) + " " + to_string(rec[1]),
                                    to_string(rec[2]) + " " + to_string(rec[1]),
                                    to_string(rec[2]) + " " + to_string(rec[3]),
                                    to_string(rec[0]) + " " + to_string(rec[3])};
            for (auto& vertex : vertexStr) {
                if (vertexSet.find(vertex) == vertexSet.end())
                    vertexSet.insert(vertex);
                else
                    vertexSet.erase(vertex);
            }
            
            sumArea += ((long)(rec[2] - rec[0])) * ((long)(rec[3] - rec[1]));
        }

        string bottomLeftStr = to_string(bottomLeftX) + " " + to_string(bottomLeftY),
                bottomRightStr = to_string(topRightX) + " " + to_string(bottomLeftY),
                topRightStr = to_string(topRightX) + " " + to_string(topRightY),
                topLeftStr = to_string(bottomLeftX) + " " + to_string(topRightY);
        if (vertexSet.size() == 4 
            && vertexSet.count(bottomLeftStr)
            && vertexSet.count(bottomRightStr)
            && vertexSet.count(topRightStr)
            && vertexSet.count(topLeftStr))
            return sumArea == (long)(topRightX - bottomLeftX) * (long)(topRightY - bottomLeftY);
        return false;
    }
};
