class Solution {
public:
    vector<int> classNums_;
    vector<int> curNums_;
    int len_;
    
    Solution(vector<int>& nums) {
        classNums_ = nums;
        curNums_ = nums;
        len_ = nums.size();
        srand(time(0));
    }
    
    vector<int> reset() {
        curNums_ = classNums_;
        return curNums_;
    }
    
    vector<int> shuffle() {
        for (int i = len_ - 1; i >= 0; i--) {
            int tempIdx = rand() % (i + 1);
            swap(curNums_[tempIdx], curNums_[i]);
        }
        return curNums_;        
    }
};

/**
 * Your Solution object will be instantiated and called as such:
 * Solution* obj = new Solution(nums);
 * vector<int> param_1 = obj->reset();
 * vector<int> param_2 = obj->shuffle();
 */
