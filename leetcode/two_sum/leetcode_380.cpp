class RandomizedSet {
public:
    unordered_map<int, int> valToIdx_;
    vector<int> valVec_;

    RandomizedSet() {
        srand(time(0));
    }
    
    bool insert(int val) {
        bool res = valToIdx_.count(val) > 0 ? false : true;

        if (res == true) {
            valVec_.push_back(val);
            valToIdx_[val] = valVec_.size() - 1;
        }
        return res;
    }
    
    bool remove(int val) {
        bool res = valToIdx_.count(val) > 0 ? true : false;

        if (res == true) {
            valToIdx_[valVec_.back()] = valToIdx_[val];
            swap(valVec_.back(), valVec_[valToIdx_[val]]);
            valVec_.pop_back();
            valToIdx_.erase(val);
        }
        return res;
    }
    
    int getRandom() {
        int randIdx = rand() % valVec_.size();
        return valVec_[randIdx];
    }
};

/**
 * Your RandomizedSet object will be instantiated and called as such:
 * RandomizedSet* obj = new RandomizedSet();
 * bool param_1 = obj->insert(val);
 * bool param_2 = obj->remove(val);
 * int param_3 = obj->getRandom();
 */
