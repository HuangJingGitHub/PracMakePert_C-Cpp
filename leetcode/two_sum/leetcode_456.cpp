class Solution {
public:
    bool find132pattern(vector<int>& nums) {
        if (nums.size() < 3)
            return false;
        
        stack<int> stk;
        int mid_entry = -1e9;
        for (int i = nums.size() - 1; i >= 0; i--) {
            if (nums[i] < mid_entry)
                return true;
            while (stk.size() > 0 && stk.top() < nums[i]) {
                mid_entry = stk.top();
                stk.pop();
            }
            stk.push(nums[i]);
        }
        return false;
    }
};
