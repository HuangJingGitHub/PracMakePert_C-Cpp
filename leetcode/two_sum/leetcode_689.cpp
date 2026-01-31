class Solution {
public:
    vector<int> maxSumOfThreeSubarrays(vector<int>& nums, int k) {
        vector<int> sum;
        int cur = 0;
        for(int i = 0; i < k; ++i){
            cur += nums[i];
        }
        sum.push_back(cur);
        for(int i = k; i < nums.size(); ++i){
            cur += nums[i] - nums[i - k];
            sum.push_back(cur);
        }
        int n = sum.size();
        vector<int> left(n, 0), right(n, n - 1);
        for(int i = 1; i < n; ++i){
            if(sum[i] > sum[left[i - 1]]) left[i] = i;
            else left[i] = left[i - 1];
        }
        for(int i = n - 2; i >= 0; --i){
            if(sum[i] >= sum[right[i + 1]]) right[i] = i;
            else right[i] = right[i + 1];
        }
        int mx = 0;
        vector<int> ans(3);
        for(int i = k; i < n - k; ++i){
            if(mx < sum[i] + sum[left[i - k]] + sum[right[i + k]]){
                mx = sum[i] + sum[left[i - k]] + sum[right[i + k]];
                ans = {left[i - k], i, right[i + k]};
            }
        }
        return ans;
    }
};
