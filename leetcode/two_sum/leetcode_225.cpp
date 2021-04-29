class MyStack {
public:
    queue<int> classQueue;
    /** Initialize your data structure here. */
    MyStack() {

    }
    
    /** Push element x onto stack. */
    void push(int x) {
        classQueue.push(x);
    }
    
    /** Removes the element on top of the stack and returns that element. */
    int pop() {
        size_t len = classQueue.size();
        for (size_t i = 0; i < len - 1; i++) { 
            classQueue.push(classQueue.front());
            classQueue.pop();
        }
        int res = classQueue.front();
        classQueue.pop();
        return res;
    }
    
    /** Get the top element. */
    int top() {
        return classQueue.back();
    }
    
    /** Returns whether the stack is empty. */
    bool empty() {
        return classQueue.empty();
    }
};
