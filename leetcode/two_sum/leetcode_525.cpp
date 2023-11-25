class Solution {
public:
    int findMaxLength(vector<int>& nums) {
        int res = 0;
        unordered_map<int, int> pre_sum_idx;
        int cnt = 0;
        pre_sum_idx[cnt] = -1;

        for (int i = 0; i < nums.size(); i++) {
            if (nums[i] == 1)
                cnt++;
            else
                cnt--;
            
            if (pre_sum_idx.count(cnt) != 0) 
                res = max(res, i - pre_sum_idx[cnt]);
            else
                pre_sum_idx[cnt] = i;
        }
        return res;
    }
};
