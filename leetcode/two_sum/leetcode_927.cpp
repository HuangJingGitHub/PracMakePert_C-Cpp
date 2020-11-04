// step 1: get ones number, if oneNum % 3 ！= 0， return {-1, -1}. Special case: all 0, return {0, S.size() -1};
// step 2: from begin and end get 2 cut positions so that each part has oneSum / 3 ones;
// step 3: get the number of zeros at end from the 3rd part;
// step 4: get the beginning positions of 2nd and 3rd parts by shifting end zeros;
// step 5: reach beginning 1 of each part and compare each digit of 3 parts.
class Solution {
public:
    vector<int> threeEqualParts(vector<int>& A) {
        vector<int> res{-1, -1};
        int oneNum = 0, pos1 = 0, pos2 = A.size() - 1;
        
        for (int x : A)
            if (x == 1)
                oneNum++;
        if (oneNum % 3 != 0)
            return res;
        else if (oneNum == 0){
            res[0] = 0;
            res[1] = A.size() - 1;
            return res;
        }
        
        int ones = oneNum / 3, endZeros = 0;
        for (int i = 0, currOne = 0; i < A.size(); i++){
            if (A[i] == 1)
                currOne++;
            if (currOne == ones){
                pos1 = i;
                break;
            }
        }
        for (int i = A.size() - 1, currOne = 0; i >= 0; i--){
            if (A[i] == 1)
                currOne++;
            if (currOne == ones){
                pos2 = i;
                break;
            }
        }


        for (int i = A.size() - 1; A[i] == 0; i--)
            endZeros++;

        for (int i = 1; i <= endZeros; i++)
            if (A[pos1 + i] != 0)
                return res;
        pos1 += endZeros;
        int zeroCnt = 1;
        for (; A[pos2 - zeroCnt] == 0; zeroCnt++) { }
        pos2 -= zeroCnt;
        
        for (int i = 1; i <= endZeros; i++)
            if (A[pos2 + i] != 0)
                return res;
        pos2 += (endZeros + 1);


        int begin1 = 0, begin2 = pos1 + 1, begin3 = pos2;
        while (A[begin1] == 0)
            begin1++;
        while (A[begin2] == 0)
            begin2++;
        while (A[begin3] == 0)
            begin3++;

        for (; begin1 <= pos1; begin1++, begin2++, begin3++)
            if (!(A[begin1] == A[begin2] && A[begin2] == A[begin3]))
                return res;
        
        res[0] = pos1;
        res[1] = pos2;
        return res;                      

    }
};
