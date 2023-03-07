class Solution {
public:
    int findKthNumber(int n, int k) {
        int cur = 1;
        k--;
        
        while (k > 0) {
            long long left = cur;
            long long right = cur + 1;
            int node_num = 0;
            
            while (left <= n) {
                node_num += min(right, (long long)(n + 1)) - left;
                left *= 10;
                right *= 10;
            }
            
            if (node_num <= k) {
                k -= node_num;
                cur++;
            }
            else {
                k--;
                cur *= 10;
            }
        }
        return cur;       
    }
};
