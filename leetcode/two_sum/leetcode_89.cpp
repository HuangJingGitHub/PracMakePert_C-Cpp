// recursion
class Solution {
public:
    vector<int> grayCode(int n) {
        if (n==0) 
            return {0};

        vector<int> res = grayCode(n-1);
        for (int i = pow(2, n-1)-1; i >= 0; i--)
            res.push_back(res[i] + pow(2, n-1));
        return res;
    }
};
