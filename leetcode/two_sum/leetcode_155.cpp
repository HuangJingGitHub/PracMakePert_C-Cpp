class MinStack {
public:
    stack<int> minStack;
    vector<int> helpVec;
    int minElement = INT_MAX;
    /** initialize your data structure here. */
    MinStack() {}
    
    void push(int x) {
        minStack.push(x);
        helpVec.push_back(x);
        if (x < minElement)
            minElement = x;
    }
    
    void pop() {
        if (minStack.top() == minElement) {
            for (int i = 0; i < helpVec.size(); i++)
                if (helpVec[i] == minStack.top()) {
                    helpVec.erase(helpVec.begin() + i);
                    break;
                }
            minElement = helpVec.front();
            for (int x : helpVec)
                minElement = min(minElement, x);
        }
        minStack.pop();
    }
    
    int top() {
        return minStack.top();
    }
    
    int getMin() {
        return minElement;
    }
};

/**
 * Your MinStack object will be instantiated and called as such:
 * MinStack* obj = new MinStack();
 * obj->push(x);
 * obj->pop();
 * int param_3 = obj->top();
 * int param_4 = obj->getMin();
 */
