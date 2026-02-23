class Solution {
public:
    bool checkValidString(string s) {
        stack<int> left;
        stack<int> star;
        
        for (int i = 0; i < s.size(); i++) {
            char c = s[i];
            if (c == '(')
                left.push(i);
            if (c == '*')
                star.push(i);
            
            if (c == ')') {
                if (!left.empty())
                    left.pop();
                else if (!star.empty())
                    star.pop();
                else
                    return false;
            }
        }

        while (!left.empty()) {
            int l_pos = left.top();
            if (star.empty())
                return false;
            int s_pos = star.top();
            if (s_pos > l_pos) {
                left.pop();
                star.pop();
            }
            else
                return false;
        }
        return true;
    }
};
