// dp
class Solution {
public:
    int minFallingPathSum(vector<vector<int>>& A) {
        int n = A.size();
        for (int i = 1; i < n; i++){
            A[i][0] += min(A[i - 1][0], A[i - 1][1]);
            A[i][n - 1] += min(A[i - 1][n - 2], A[i - 1][n - 1]);            
            for (int j  = 1; j < n - 1; j++){
                int preRowMin = min(A[i - 1][j - 1], A[i - 1][j]);
                preRowMin = min(preRowMin, A[i - 1][j + 1]);
                A[i][j] += preRowMin;
            }
        }
        return *min_element(A.back().begin(), A.back().end());
    }
};
