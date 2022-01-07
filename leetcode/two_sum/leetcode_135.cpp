// interesting problem
class Solution {
public:
    int candy(vector<int>& ratings) {
        if (ratings.empty())
            return 0;
        
        vector<int> candyNum(ratings.size(), 0);
        candyNum[0] = 1;
        for (int i = 1; i < ratings.size(); i++){  // one loop to deal with left relationship
            if (ratings[i] > ratings[i - 1])
                candyNum[i] = candyNum[i - 1] + 1;
            else
                candyNum[i] = 1;
        }
        for (int i = ratings.size() - 2; i >= 0; i--){ // one loop to deal with right relationship
            if (ratings[i] > ratings[i + 1])
                candyNum[i] = max(candyNum[i], candyNum[i + 1] + 1);
        }

        int res = 0;
        for (int x : candyNum)
            res += x;
        return res;
    }
};


// Algorithm is correct, but O(n^2) complexity
class Solution {
public:
    int candy(vector<int>& ratings) {
        if (ratings.empty() == true)
            return 0;

        int res = 1, preCandy = 1;
        vector<int> candyVec(ratings.size(), 1);
        for (int i = 1; i < ratings.size(); i++) {
            if (ratings[i] == ratings[i - 1]) {
                res += 1;
                preCandy = 1;
            }
            else if (ratings[i] < ratings[i - 1]) {
                if (preCandy == 1) {
                    res += 1;
                    candyVec[i] = 1;
                    for (int j = i - 1; j >= 0 && ratings[j] > ratings[j + 1] 
                        && candyVec[j] <= candyVec[j + 1]; j--) {
                        res += 1;
                        candyVec[j]++;
                    }
                    preCandy = 1;
                }
                else {
                    res += 1;
                    preCandy = 1;
                }
            }
            else {
                res += preCandy + 1;
                preCandy++;
                candyVec[i] = preCandy;
            }
        }

        return res;
    }
};
