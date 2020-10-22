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
