public:
    bool validateStackSequences(vector<int>& pushed, vector<int>& popped) {
        if (pushed.size() <= 1)
            return true;

        stack<int> testStack;
        int len = pushed.size(), pushPtr = 0, popPtr = 0;
        testStack.push(pushed[pushPtr++]);

        while (pushPtr < len){
            while (testStack.empty() || pushPtr < len && testStack.top() != popped[popPtr]){
                testStack.push(pushed[pushPtr]);
                pushPtr++;
            }
            while (!testStack.empty() && popPtr < len && testStack.top() == popped[popPtr]){
                testStack.pop();
                popPtr++;
            }
        }

        
        return testStack.empty();
    }
};

// --> Same method, but just more compact and elegent in the loop
class Solution {
public:
    bool validateStackSequences(vector<int>& pushed, vector<int>& popped) {
        if (pushed.size() <= 1)
            return true;

        stack<int> testStack;
        int len = pushed.size(), popPtr = 0;

        for (int i = 0; i < len; i++){
            testStack.push(pushed[i]);
            while (!testStack.empty() && testStack.top() == popped[popPtr]){
                testStack.pop();
                popPtr++;
            }
        }
     
        return testStack.empty();
    }
};
