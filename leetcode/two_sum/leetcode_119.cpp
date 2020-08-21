// recursion or use pre & cur variables and keep updating
class Solution {
public:
    vector<int> getRow(int rowIndex) {
        if (rowIndex == 0 || rowIndex == 1)
            return vector<int>(rowIndex + 1, 1);
        
        vector<int> res(rowIndex + 1, 1), lsatRes = getRow(rowIndex-1);
        for (int i = 1; i < rowIndex; i++)
            res[i] = lsatRes[i-1] + lsatRes[i];
        
        return res;
    }
};
