class Solution {
public:
    vector<vector<vector<int>>> dp;
    int removeBoxes(vector<int>& boxes) {
        dp = vector<vector<vector<int>>>(boxes.size(), vector<vector<int>>(boxes.size(), vector<int>(boxes.size(), 0)));
        return caculate(boxes, 0, boxes.size() - 1, 0);
    }

    int caculate(vector<int>& boxes, int l, int r, int k) {
        if (l > r)
            return 0;
        
        if (dp[l][r][k] != 0)
            return dp[l][r][k];
        
        dp[l][r][k] = caculate(boxes, l, r - 1, 0) + (k + 1) * (k + 1);
        for (int i = l; i < r; i++) {
            if (boxes[i] == boxes[r])
                dp[l][r][k] = std::max(dp[l][r][k], caculate(boxes, l, i, k + 1) + caculate(boxes, i + 1, r - 1, 0));
        }
        return dp[l][r][k];
    }
};
