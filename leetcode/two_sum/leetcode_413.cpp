class Solution {
public:
    int numberOfArithmeticSlices(vector<int>& A) {
        if (A.size() <= 2)
            return 0;

        int res = 0;        
        vector<int> dif(A.size() - 1), groupNum;
        for (int i = 0; i < A.size() - 1; i++)
            dif[i] = A[i + 1] - A[i];

        for (int i = 1, num = 1; i < dif.size(); i++) {
            if (dif[i] == dif[i - 1])
                num++;
            if (dif[i] != dif[i - 1] || i == dif.size() - 1) {
                if (num >= 2)
                    groupNum.push_back(num + 1);
                num = 1;
            }
        }
        
        for (int num : groupNum)
            res += (num - 1) * (num - 2) / 2;
        return res;
    }
};
