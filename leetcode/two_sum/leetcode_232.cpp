class MyQueue {
public:
    stack<int> classStack;
    stack<int> storeStack;
    /** Initialize your data structure here. */
    MyQueue() { }
    
    /** Push element x to the back of queue. */
    void push(int x) {
        classStack.push(x);
    }
    
    /** Removes the element from in front of queue and returns that element. */
    int pop() {
        int res;
        while (!classStack.empty()) {
            storeStack.push(classStack.top());
            classStack.pop();
        }
        res = storeStack.top();

        storeStack.pop();
        while (!storeStack.empty()) {
            classStack.push(storeStack.top());
            storeStack.pop();
        }
        return res;
    }
    
    /** Get the front element. */
    int peek() {
        int res;
        while (!classStack.empty()) {
            storeStack.push(classStack.top());
            classStack.pop();
        }
        res  = storeStack.top();

        while (!storeStack.empty()) {
            classStack.push(storeStack.top());
            storeStack.pop();
        }
        return res;
    }
    
    /** Returns whether the queue is empty. */
    bool empty() {
        return classStack.empty();
    }
};
