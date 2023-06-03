class Solution {
public:
    bool circularArrayLoop(vector<int>& nums) {
        int n = nums.size();

        int cur = 0, next = 0;
        for (int start = 0, size_cnt = 0; start < nums.size(); ) {
            cur = next;
            next = (cur + (nums[cur] % n) + n) % n;  
            size_cnt++;

            if (size_cnt > n || nums[cur] * nums[next] < 0 || next == cur) {
                next = ++start;
                size_cnt  = 0;
            }
            else if (next == start)
                return true;
        }
        return false;
    }
};
