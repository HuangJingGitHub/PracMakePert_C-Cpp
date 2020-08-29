class Solution {
public:
    int maxProfit(vector<int>& prices) {
        if (prices.size() < 2)
            return 0;
            
        int res = 0;
        int salePrice = prices.back();

        for (int i = prices.size()-2; i >= 0; i--){
            int previousPrice = prices[i];
            if (previousPrice < salePrice){
                int profit = salePrice - previousPrice;
                res = (profit > res) ? profit : res;
            }
            else if (previousPrice > salePrice){
                salePrice = previousPrice;
            }
        }

        return res;
    }
};
