class Solution {
public:
    int maxSideLength(vector<vector<int>>& mat, int threshold) {
        if (mat.empty())
            return 0;

        int integralSum = 0, res = 0, minDim = min(mat.size(), mat[0].size());
        
        for (int i = 0; i < minDim; i++){
            updateSquareSum(mat, integralSum, i);
            if (integralSum > threshold)
                return i;
        }
        return minDim;
    }

    void updateSquareSum(vector<vector<int>>& mat, int& integralSum, int idxLen)
    {
        for (int i = 0; i <= idxLen; i++){
            integralSum += mat[idxLen][i];
            integralSum += mat[i][idxLen];
        }
            integralSum -= mat[idxLen][idxLen];
    }
};
