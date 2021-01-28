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
