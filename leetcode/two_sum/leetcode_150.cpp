class Solution {
public:
    int evalRPN(vector<string>& tokens) {
        stack<int> numStack;
        
        for (int i = 0; i < tokens.size(); i++) {
            if (tokens[i] == "+" || tokens[i] == "-" || tokens[i] == "*" || tokens[i] == "/" ) {
                int operand1, operand2;
                operand2 = numStack.top();
                numStack.pop();
                operand1 = numStack.top();
                numStack.pop();

                if (operation == "+")
                    numStack.push(operand1 + operand2);
                else if (operation == "-")
                    numStack.push(operand1 - operand2);
                else if (operation == "*")
                    numStack.push(operand1 * operand2);
                else   
                    numStack.push(operand1 / operand2);                                
            }
            else 
                numStack.push(stoi(tokens[i]));
        }
        
        return numStack.top();
    }
};
