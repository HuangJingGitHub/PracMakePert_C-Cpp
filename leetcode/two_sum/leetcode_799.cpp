class Solution {
public:
    double champagneTower(int poured, int query_row, int query_glass) {
        vector<double> rowState(1, poured);

        for (int row = 0; row < query_row; row++) {
            vector<double> nextRowState(row + 2, 0);
            for (int i = 0; i < rowState.size(); i++) {
                if (rowState[i] > 1) {
                    nextRowState[i] += (rowState[i] - 1) / 2;
                    nextRowState[i + 1] += (rowState[i] - 1) / 2;
                }
            }
            rowState = nextRowState;
        }

        if (rowState[query_glass] > 1)
            return 1;
        return rowState[query_glass];
    }
};
