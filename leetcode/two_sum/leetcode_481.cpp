class Solution {
public:
    int magicalString(int n) {
        vector<int> numVec(n + 1, 0);
        numVec[0] = 1;
        numVec[1] = 2;

        bool isOne = true;
        int slowIdx = 0, fastIdx = 0, res = 0;
        while (fastIdx < n) {
            int curOccNum = numVec[slowIdx++];
            if (isOne == true && curOccNum == 1) {
                numVec[fastIdx++] = 1;
                res += 1;
            }
            else if (isOne == true && curOccNum == 2) {
                numVec[fastIdx++] = 1;
                numVec[fastIdx++] = 1;
                res += 2;
            }
            else if (isOne == false && curOccNum == 1) {
                numVec[fastIdx++] = 2;   
            }
            else {
                numVec[fastIdx++] = 2;
                numVec[fastIdx++] = 2;
            }
            isOne = !isOne;
        }

        if (numVec.back() == 1)
            res--;

        return res;
    }
};
