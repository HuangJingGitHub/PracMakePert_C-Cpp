class Solution {
public:
    int maxProfit(vector<int>& prices) {
        if (prices.size() < 2)
            return 0;
            
        int res = 0;
        int salePrice = prices.back();

        for (int i = prices.size()-2; i >= 0; i--){
            if (prices[i] < salePrice){
                int profit = salePrice - prices[i];
                res = (profit > res) ? profit : res;  // or res = max(profit, res);
            }
            else if (prices[i] > salePrice){
                salePrice = prices[i];
            }
        }

        return res;
    }
};

class Solution {
public:
    int maxProfit(vector<int>& prices) {
        if (prices.size() < 2)
            return 0;
            
        int res = 0, buyPrice = prices[0];
        for (int i = 1; i < prices.size(); i++) {
            if (prices[i] > buyPrice) {
                res = max(res, prices[i] - buyPrice);
            }
            else
                buyPrice = prices[i];
        }

        return res;
    }
};

// dp style
class Solution {
public:
    int maxProfit(vector<int>& prices) {
        if (prices.size() < 2)
            return 0;

        int res = 0, preMin = prices[0], preRes = 0;
        for (int i = 1; i < prices.size(); i++) {
            if (prices[i] > preMin) {
                res = max(preRes, prices[i] - preMin);
                preRes = res;
            }
            else
                preMin = prices[i];
        }

        return res;
    }
};
