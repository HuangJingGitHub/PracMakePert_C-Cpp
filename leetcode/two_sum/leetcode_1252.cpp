class Solution {
public:
    int oddCells(int m, int n, vector<vector<int>>& indices) {
        int res = 0;
        vector<int> addTimesRow(m, 0), addTimesCol(n, 0);
        for (auto& indixPair : indices) {
            addTimesRow[indixPair[0]]++;
            addTimesCol[indixPair[1]]++;
        }

        int addTimesOddRow = 0, addTimesEvenRow = 0;
        for (int i = 0; i < m; i++) {
            if (addTimesRow[i] % 2 == 0)
                addTimesEvenRow++;
            else
                addTimesOddRow++;
        }

        for (int i = 0; i < n; i++) {
            if (addTimesCol[i] % 2 == 0)
                res += addTimesOddRow;
            else
                res += addTimesEvenRow;
        }
        return res;
    }
};
