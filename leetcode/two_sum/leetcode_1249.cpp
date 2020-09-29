// No difference with parenthese checking problem
class Solution {
public:
    string minRemoveToMakeValid(string s) {
        stack<int> idxStack;

        for (int i = 0; i < s.size(); i++){
            if (s[i] == '(')
                idxStack.push(i);
            else if (s[i] == ')'){
                if (!idxStack.empty() && s[idxStack.top()] == '(')
                    idxStack.pop();
                else
                    idxStack.push(i);
            }
        }

        while (!idxStack.empty()){
            s.erase(idxStack.top(), 1);
            idxStack.pop();
        }

        return s;
    }
};
