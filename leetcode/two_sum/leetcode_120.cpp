// classical dp
class Solution {
public:
    int minimumTotal(vector<vector<int>>& triangle) {
        int height = triangle.size();

        for(int i = 1; i < height; i++)
            triangle[i][0] += triangle[i-1][0];

        for (int i = 1; i < height; i++){
            for (int j = 1; j < triangle[i].size()-1; j++){
                triangle[i][j] += min(triangle[i-1][j-1], triangle[i-1][j]);
            }
            triangle[i].back() += triangle[i-1].back();
        }
        return *min_element(triangle[height-1].begin(), triangle[height-1].end());
    }
};
