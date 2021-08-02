// exceed time limit
class NumArray {
private:
    vector<int> nums_;
    vector<int> sum_;
public:
    NumArray(vector<int>& nums) {
        nums_ = nums;
        sum_ = nums;
        for (int i = 1; i < sum_.size(); i++)
            sum_[i] += sum_[i - 1];
    }
    
    void update(int index, int val) {
        if (index < 0 || index >= nums_.size())
            return;

        int variation = -nums_[index] + val;
        nums_[index] = val;
        for (int i = index; i < sum_.size(); i++)
            sum_[i] += variation;
    }
    
    int sumRange(int left, int right) {
        if (left == 0)
            return sum_[right];
        return sum_[right] - sum_[left - 1];
    }
};

/**
 * Your NumArray object will be instantiated and called as such:
 * NumArray* obj = new NumArray(nums);
 * obj->update(index,val);
 * int param_2 = obj->sumRange(left,right);
 */
