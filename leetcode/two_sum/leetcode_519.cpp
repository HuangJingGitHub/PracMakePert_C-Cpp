class Solution {
public:
    unordered_map<int, int> map_;
    int size_;
    int row_num_;
    int col_num_;

    Solution(int m, int n) {
        size_ = m * n;
        row_num_ = m;
        col_num_ = n;
        srand(time(NULL));
    }
    
    vector<int> flip() {
        int random_indix = rand() % size_;
        int temp = random_indix;
        while (map_.count(temp) == 1) 
            temp = map_[temp];
        map_[random_indix] = size_ - 1;
        size_--;
        return {temp / col_num_, temp % col_num_};
    }
    
    void reset() {
        map_.clear();
        size_ = row_num_ * col_num_;
    }

};

/**
 * Your Solution object will be instantiated and called as such:
 * Solution* obj = new Solution(m, n);
 * vector<int> param_1 = obj->flip();
 * obj->reset();
 */
