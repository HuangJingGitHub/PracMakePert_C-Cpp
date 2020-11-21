class Solution {
public:
    int binaryGap(int n) {
        int res = 0;
        vector<int> bin, oneIdx;
        while (n / 2 != 0){
            bin.push_back(n % 2);
            n /= 2;
        }
        bin.push_back(1);
        // reverse(bin.begin(), bin.end());
        for (int i = 0; i < bin.size(); i++)
            if (bin[i] == 1)
                oneIdx.push_back(i);
        for (int i = 0; i < oneIdx.size() - 1; i++)
            res = max(res, oneIdx[i + 1] - oneIdx[i]);
        return res;
    }
};
