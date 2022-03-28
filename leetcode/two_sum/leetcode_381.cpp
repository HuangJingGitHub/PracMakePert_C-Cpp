#include<vector>
class RandomizedCollection {
public:
    map<int, vector<int>> valToIdx_;
    vector<int> storeVec_;

    RandomizedCollection() {
        srand(time(0));
    }
    
    bool insert(int val) {
        bool res;
        if (valToIdx_.count(val) == 0 
            || (valToIdx_.count(val) != 0 && valToIdx_[val].size() == 0)) {
            res = true;
        }
        else {
            res = false;
        }
        storeVec_.push_back(val);
        valToIdx_[val].push_back(storeVec_.size() - 1);       
        return res;
    }
    
    bool remove(int val) {      
        bool res = true;      
        if (valToIdx_.count(val) > 0 && valToIdx_[val].size() > 0) {
            int tempIdx = valToIdx_[val].back(), lastVal = storeVec_.back();
            for (auto it = valToIdx_[lastVal].begin(); it != valToIdx_[lastVal].end(); it++)  // If set is used to store the indices, it will automatically sort the members.
                if (*it == storeVec_.size() - 1) {
                    valToIdx_[lastVal].erase(it);
                    break;
                }
            valToIdx_[lastVal].push_back(tempIdx);
            valToIdx_[val].pop_back();
            swap(storeVec_[tempIdx], storeVec_.back());             
            storeVec_.pop_back();
        }
        else 
            res = false;
        return res;
    }
    
    int getRandom() {
        int randIdx = rand() % (int)(storeVec_.size());
        return storeVec_[randIdx];
    }
};
