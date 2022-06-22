const int N = 100010;
class Solution {
public:
    int b[N];
    void insert(int l, int r) {
        b[l] += 1, b[r + 1] -= 1;
    }

    int bestRotation(vector<int>& nums) {
        int n = nums.size();
        for (int i = 0; i < n; i++) {
            int left = i + 1, right = (n + i - nums[i]) % n + 1;
            if (left <= right) 
                insert(left, right);
            else {
                insert(1, right);
                insert(left, n);
            }
        }
        int res, score = 0, max_score = 0;
        for (int i = 1; i <= n; i++) {
            score += b[i];
            if (score > max_score) { 
                res = i - 1; 
                max_score = score; 
            }
        }
        return res;
    }
};
