class Solution {
public:
    vector<vector<int>> generateMatrix(int n) {
        vector<vector<int>> res;
        vector<int> onesRow(n, 1);
        for (int i = 0; i < n; i++)
            res.push_back(onesRow);

        int row = 0, col = 0, num = 1;
        for (int i = 0; i < n-1; i++){
            for (row = i, col = i; col < n - i; col++, num++)
                res[row][col] = num;
            for (row++, col--; row < n - i; row++, num++)
                res[row][col] = num;
            for (row--, col--; col >= i; col--, num++)
                res[row][col] = num;
            for (row--,col++; row > i; row--, num++)
                res[row][col] = num;
        }
        return res;
    }
};
