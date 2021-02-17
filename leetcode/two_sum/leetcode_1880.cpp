class TripleInOne {
private:
    vector<int> classVec;   
    int stackSize_; 
    int idx0;
    int idx1;
    int idx2;
public:
    TripleInOne(int stackSize) {
        classVec = vector<int>(stackSize * 3);
        stackSize_ = stackSize;
        idx0 = -1;
        idx1 = stackSize - 1;
        idx2 = stackSize * 2 - 1;
    }
    
    void push(int stackNum, int value) {
        if (stackNum == 0) {
            if (idx0 == stackSize_ - 1)
                return;
            else 
                classVec[++idx0] = value;
        }
        else if (stackNum == 1) {
            if (idx1 == stackSize_ * 2 - 1)
                return;
            else
                classVec[++idx1] = value;
        }
        else if (stackNum == 2) {
            if (idx2 == stackSize_ * 3 - 1)
                return;
            else
                classVec[++idx2] = value;
        }
    }
    
    int pop(int stackNum) {
        int res = -1;
        if (stackNum == 0) {
            if (idx0 == -1)
                return -1;
            else 
                res = classVec[idx0--];
        }
        else if (stackNum == 1) {
            if (idx1 == stackSize_ - 1)
                return -1;
            else
                res = classVec[idx1--];
        }
        else if (stackNum == 2) {
            if (idx2 == stackSize_ * 2 - 1)
                return -1;
            else 
                res = classVec[idx2--];
        }
        return res;
    }
    
    int peek(int stackNum) {
        int res = -1;
        if (stackNum == 0) {
            if (idx0 == -1)
                return -1;
            else
                res = classVec[idx0];
        }
        else if (stackNum == 1) {
            if (idx1 == stackSize_ - 1)
                return -1;
            else 
                res = classVec[idx1];
        }
        else if (stackNum == 2) {
            if (idx2 == stackSize_ * 2 - 1)
                return -1;
            else
                res = classVec[idx2];
        }
        return res;
    }
    
    bool isEmpty(int stackNum) {
        bool res = true;
        if (stackNum == 0)
            res = (idx0 == -1);
        else if (stackNum == 1)
            res = (idx1 == stackSize_ - 1);
        else if (stackNum == 2)
            res = (idx2 == stackSize_ * 2 - 1);
        return res;
    }
};

/**
 * Your TripleInOne object will be instantiated and called as such:
 * TripleInOne* obj = new TripleInOne(stackSize);
 * obj->push(stackNum,value);
 * int param_2 = obj->pop(stackNum);
 * int param_3 = obj->peek(stackNum);
 * bool param_4 = obj->isEmpty(stackNum);
 */
