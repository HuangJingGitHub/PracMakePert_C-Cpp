// dynamic programming
class Solution {
public:
    int maxDotProduct(vector<int>& nums1, vector<int>& nums2) {
        int len1 = nums1.size(), len2 = nums2.size();
        vector<vector<int>> dp(len1, vector<int>(len2, 0));
        
        dp[0][0] = nums1[0] * nums2[0];
        for (int i = 1; i < len2; i++)  // First row
            dp[0][i] = max(dp[0][i-1], nums1[0]*nums2[i]);
        for (int i = 1; i < len1; i++)  // First column
            dp[i][0] = max(dp[i-1][0], nums1[i]*nums2[0]);
        
        // For the state transition function, what is trivial is when nums1[i] and nums2[j] are both chosen,
        // dp[i][j] is max(nums1[i]*nums2[j], dp[i-1][j-1] + nums1[i]*nums2[j]) as there can be no previous elements chosen. 
        for (int i = 1; i < len1; i++)
            for (int j = 1; j < len2; j++){
                dp[i][j] = max_5(dp[i-1][j-1], nums1[i]*nums2[j], dp[i-1][j-1] + nums1[i]*nums2[j], dp[i-1][j], dp[i][j-1]);
            }
        return dp[len1-1][len2-1];
    }

    int max_5(int x1, int x2, int x3, int x4, int x5){
        int temp1 = max(x1, x2), temp2 = max(x3, x4);
        int temp3 = max(temp1, temp2);
        return max(x5, temp3);
    }
};
