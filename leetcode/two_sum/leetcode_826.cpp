// using property of sorted array
class Solution {
public:
    static bool cmp(const pair<int, int>& a, const pair<int, int>& b){
        return a.first < b.first;
    }

    int maxProfitAssignment(vector<int>& difficulty, vector<int>& profit, vector<int>& worker) {
        int res = 0, currProfit = 0;
        vector<pair<int, int>> profitDifficulty;

        for (int i = 0; i < difficulty.size(); i++)
            profitDifficulty.push_back(pair(difficulty[i], profit[i]));

        sort(profitDifficulty.begin(), profitDifficulty.end(), cmp);
        sort(worker.begin(), worker.end());

        for (int i = 0, j = 0; i < worker.size(); i++){
            while (j < profitDifficulty.size() && profitDifficulty[j].first <= worker[i]){
                currProfit = max(currProfit, profitDifficulty[j].second);
                j++;
            }
            res += currProfit;
        }
        return res;
    }
};
