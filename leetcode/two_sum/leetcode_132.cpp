// dp solution
class Solution {
public:
    int minCut(string s) {
        if (s.size() < 2)
            return 0;
        
        vector<int> dp(s.size() + 1, 0);
        dp[0] = -1;  // Set the first element for no string case "" for operation at position 0, for consistency.
        for (int i = 1; i < s.size(); i++){
            dp[i + 1] = dp[i] + 1;
            for(int j = 0; j < i; j++){
                if (isPalindrome(s, j, i))
                    dp[i + 1] = min(dp[i + 1], dp[j] + 1);
            }
        }
        return dp.back();
    }

    bool isPalindrome(const string& s, int start, int end){
        if (start > end || start >= s.size() || end < 0)
            return false;
        
        while (start < end){
            if (s[start] != s[end])
                return false;
            start++;
            end--;
        }
        return true;
    }
};
