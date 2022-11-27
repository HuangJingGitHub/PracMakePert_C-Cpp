class Solution {
public:
    bool canCross(vector<int>& stones) {
        if (stones.size() <= 2)
            return true;
        
        vector<bool> dp(stones.size(), false);
        dp[0] = dp[1] = true;

        for (int i = 2; i < stones.size(); i++) {
            
        }
    }
};
