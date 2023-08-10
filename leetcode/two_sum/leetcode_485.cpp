class Solution {
public:
    int findMaxConsecutiveOnes(vector<int>& nums) {
        int res = nums[0] == 1 ? 1 : 0;
        int cnt = res;
        
        for (int i = 1; i < nums.size(); i++) {
            if (nums[i] == 0)
                cnt = 0;
            else 
                res = max(res, ++cnt);
        }

        return res;
    }
};
