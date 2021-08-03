// see the discussion part, very cool
class Solution {
public:
    int maxProfit(vector<int>& prices) {
        if (prices.size() <= 1)
            return 0;
        
        vector<int> sell(prices.size(), 0), buy(prices.size(), 0), cool(prices.size(), 0);
        buy[0] = -prices[0];

        for (int i = 1; i < prices.size(); i++) {
            sell[i] = max(buy[i - 1] + prices[i], sell[i - 1]);
            buy[i] = max(cool[i - 1] - prices[i], buy[i - 1]);
            int temp = max(sell[i - 1], buy[i - 1]);
            cool[i] = max(temp, cool[i - 1]);
        }

        return max(sell.back(), cool.back());
    }
};
