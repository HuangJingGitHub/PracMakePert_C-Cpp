class Solution {
public:
    vector<int> sequentialDigits(int low, int high) {
        vector<int> res;
        int lowDigitNum = 1, highDigitNum = 1;
        for (int i = low / 10; i != 0; i /= 10)
            lowDigitNum++;
        for (int i = high / 10; i != 0; i /= 10)
            highDigitNum++;

        for (int i = lowDigitNum; i <= highDigitNum; i++){
            for (int j = 1; j <= 10 - i; j++){
                int sequentialNum = j;
                for (int k = 1; k < i; k++)
                    sequentialNum = sequentialNum * 10 + j + k;
                if (sequentialNum >= low && sequentialNum <= high)
                    res.push_back(sequentialNum);
            }
        }

        return res;
    }
};
