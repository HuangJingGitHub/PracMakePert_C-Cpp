class Solution {
public:
    int coinChange(vector<int>& coins, int amount) {
        int res = 0, curAmount = 0;
        
        sort(coins.begin(), coins.end());
        reverse(coins.begin(), coins.end());
        vector<int> maxCoinNum(coins.size()), curCoinNum(coins.size());
        
        for (int i = 0; i < maxCoinNum.size(); i++)
            maxCoinNum[i] = amount / coins[i];
        
        while (curAmount != amount) {
            
        }
        
    }
};
