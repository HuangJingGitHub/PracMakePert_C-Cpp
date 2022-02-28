class Solution {
public:
    bool isValid(string s) {
        if (s.size() == 0)
            return true;
            
        vector<char> stackStr;
        stackStr.push_back(s[0]);

        for (int i = 1; i < s.length(); i++)
        {
            switch(s[i])
            {
                case '(':
                    stackStr.push_back(s[i]);
                    break;
                case '{':
                    stackStr.push_back(s[i]);
                    break;
                case '[':
                    stackStr.push_back(s[i]);
                    break;

                case ')':
                    if (stackStr.size() > 0 && stackStr[stackStr.size() - 1] == '(')
                        stackStr.pop_back();
                    else
                        stackStr.push_back(s[i]);
                    break;
                case '}':
                    if (stackStr.size() > 0 && stackStr[stackStr.size() - 1] == '{')
                        stackStr.pop_back();
                    else
                        stackStr.push_back(s[i]);
                    break;
                case ']':
                    if (stackStr.size() > 0 && stackStr[stackStr.size() - 1] == '[')
                        stackStr.pop_back();
                    else
                        stackStr.push_back(s[i]);   
                    break;                                             
            }
        }
        return stackStr.size() == 0;
    }
};


class Solution {
public:
    bool isValid(string s) {
        if (s.size() == 0)
            return true;
        
        stack<char> rightStk;
        rightStk.push(s.back());

        for (int i = s.size() - 2; i >= 0; i--) {
            if (s[i] == '(') {
                if (rightStk.empty() == false && rightStk.top() == ')')
                    rightStk.pop();
                else 
                    rightStk.push('(');
            }
            else if (s[i] == '[') {
                if (rightStk.empty() == false && rightStk.top() == ']')
                    rightStk.pop();
                else
                    rightStk.push('[');
            }
            else if (s[i] == '{') {
                if (rightStk.empty() == false && rightStk.top() == '}')
                    rightStk.pop();
                else
                    rightStk.push('{');
            }
            else 
                rightStk.push(s[i]);
        }
        return rightStk.empty();
    }
};
