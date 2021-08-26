class Solution {
public:
    vector<int> maxNumber(vector<int>& nums1, vector<int>& nums2, int k) {
        vector<int> res(k, 0);

        int ptr1 = 0, ptr2 = 0, n1 = nums1.size(), n2 = nums2.size(), i = 0;
        for (; i < k; i++) {
            int leftDigitsNum = k - i;
            int minDigitsNumFromNums1 = (leftDigitsNum - (n2 - ptr2) <= 0) ? 0 : leftDigitsNum - (n2 - ptr2),
                minDigitsNumFromNums2 = (leftDigitsNum - (n1 - ptr1) <= 0) ? 0 : leftDigitsNum - (n1 - ptr1);

            int maxIdx1 = ptr1, curMax1 = nums1[ptr1], searchEnd1 = n1 - minDigitsNumFromNums1;
            if (minDigitsNumFromNums1 == 0)
                searchEnd1 = n1 - 1;
            for (int j = ptr1 + 1; j <= searchEnd1; j++)
                if (nums1[j] > curMax1) {
                    maxIdx1 = j;
                    curMax1 = nums1[j];
                }

            int maxIdx2 = ptr2, curMax2 = nums2[ptr2], searchEnd2 = n2 - minDigitsNumFromNums2;
            if (minDigitsNumFromNums2 == 0)
                searchEnd1 = n2 - 1;
            for (int j = ptr2 + 1; j <= searchEnd2; j++)
                if (nums2[j] > curMax2) {
                    maxIdx2 = j;
                    curMax2 = nums2[j];
                }
            
            res[i] = max(curMax1, curMax2);
            if (curMax1 > curMax2)
                ptr1 = maxIdx1 + 1;
            else if (curMax1 < curMax2)
                ptr2 = maxIdx2 + 1;
            else {
                //
            }
            if (ptr1 >= n1 || ptr2 >= n2)
                break;
        }

        if (ptr1 >= n1)
            for (i++; i < k; i++)
                res[i] = nums2[ptr2++];
        else
            for (i++; i < k; i++)
                res[i] = nums1[ptr1++];
        return res;
    }
};
