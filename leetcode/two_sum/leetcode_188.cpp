// Good idea to use sorting algorithm, but not working as the fundamental thought is not complete to implement the time order.
// Though this is equal to finding k pairs with max difference sum, it has transaction order limit.
class Solution {
public:
    static bool cmp(const pair<int, int>& a, const pair<int, int>& b) {
        return a.first < b.first;
    }

    int maxProfit(int k, vector<int>& prices) {
        int res = 0, transaction = 0;
        vector<pair<int, int>> marketHistory;
        vector<bool> transacted(prices.size(), false);
        for (int i = 0; i < prices.size(); i++) 
            marketHistory.push_back(pair(prices[i], i));
        sort(marketHistory.begin(), marketHistory.end(), cmp);
        for (auto item : marketHistory)
            cout << item.first << ' ';
        cout << '\n';
        for (auto item : marketHistory)
            cout << item.second << ' ';
        for (int i = 0; i < marketHistory.size() && transaction < k; i++)
            for (int j = marketHistory.size() - 1; j > i; j--) {
                if (transacted[j])
                    continue;
                else if (marketHistory[j].second < marketHistory[i].second)
                    continue;

                res += marketHistory[j].first - marketHistory[i].first;
                transacted[i] = true;
                transacted[j] = true;
                transaction++;
                break;
            }
        return res;
    }
};

// 3D dp
class Solution {
public:
    int maxProfit(int k, vector<int>& prices) {
        int res = 0;
        if (k >= prices.size() / 2) {
            for (int i = 1; i < prices.size(); i++)
                if (prices[i] > prices[i - 1])
                    res += (prices[i] - prices[i - 1]);
            return res;
        }

        vector<vector<vector<int>>> dp(prices.size(), vector<vector<int>>(k+1, vector<int>(2)));    // profit of dp[i][j][0] i-th day with j times transaction / buy, no stock
        for (int i = 0; i < prices.size(); i++) {                                                   // profit of dp[i][j][0] i-th day with j times transaction / buy, with stock
            dp[i][0][0] = 0;
            dp[i][0][1] = -INT_MAX;
        }
        for (int j = 1; j <= k; j++) {
            dp[0][j][0]  = 0;
            dp[0][j][1] = -prices[0];
        }
        for (int i = 1; i < prices.size(); i++)
            for (int j = 1; j <= k; j++) {
                dp[i][j][0] = max(dp[i-1][j][0], dp[i-1][j][1] + prices[i]);
                dp[i][j][1] = max(dp[i-1][j][1], dp[i-1][j-1][0] - prices[i]);
            }
        return dp.back().back()[0];
    }
};
