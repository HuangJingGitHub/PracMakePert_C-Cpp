class NumArray {
public:
    vector<long long int> accumulatedSum;
    NumArray(vector<int>& nums) {
        accumulatedSum = vector<long long int>(nums.size(), 0);
        accumulatedSum[0] = nums[0];
        for (int i = 1; i < nums.size(); i++)
            accumulatedSum[i] = accumulatedSum[i - 1] + nums[i];
    }
    
    int sumRange(int left, int right) {
        if (left == 0)
            return accumulatedSum[right];
        return accumulatedSum[right] - accumulatedSum[left - 1];
    }
};

/**
 * Your NumArray object will be instantiated and called as such:
 * NumArray* obj = new NumArray(nums);
 * int param_1 = obj->sumRange(left,right);
 */
