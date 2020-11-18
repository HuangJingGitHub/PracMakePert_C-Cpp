class Solution {
public:
    vector<vector<int>> reconstructMatrix(int upper, int lower, vector<int>& colsum) {
        int n = colsum.size(), sumRow0 = 0, sumRow1 = 0;
        vector<vector<int>> res(2, vector<int>(n, 0));

        for (int i = 0; i < n; i++){
            if (colsum[i] == 0){
                res[0][i] = 0;
                res[1][i] = 0;
            }
            else if (colsum[i] == 2){
                res[0][i] = 1;
                res[1][i] = 1;
                sumRow0++;
                sumRow1++;
            }
            else{    
                res[0][i] = 1;
                sumRow0++;
            }
        }
        
        if (!(sumRow0 - upper >= 0 && sumRow1 - lower <= 0 && sumRow0 + sumRow1 == upper + lower))
            return vector<vector<int>>();
        else if (sumRow0 == upper && sumRow1 == lower)
            return res;
        
        int redunant = sumRow0 - upper;
        for (int i = 0, j = 0; i < n && j < redunant; i++){
            if (colsum[i] == 1){
                res[0][i] = 0;
                res[1][i] = 1;
                sumRow1++;
                j++;
            }
        }
        return res;
    }
};
