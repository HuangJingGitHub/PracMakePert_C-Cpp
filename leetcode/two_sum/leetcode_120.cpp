\\ classical dp
class Solution {
public:
    int minimumTotal(vector<vector<int>>& triangle) {
        int m = triangle.size();

        for(int i = 1; i < m; i++)
            triangle[i][0] += triangle[i-1][0];

        for (int i = 1; i < m; i++){
            for (int j = 1; j < triangle[i].size()-1; j++){
                triangle[i][j] += min(triangle[i-1][j-1], triangle[i-1][j]);
            }
            triangle[i].back() += triangle[i-1].back();
        }
        return *min_element(triangle[m-1].begin(), triangle[m-1].end());
    }
};
