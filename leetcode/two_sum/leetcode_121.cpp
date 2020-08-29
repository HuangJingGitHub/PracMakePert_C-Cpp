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
