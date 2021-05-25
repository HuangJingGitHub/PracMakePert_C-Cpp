class Solution {
public:
    vector<int> diffWaysToCompute(string expression) {
        int cnt = 0;
        for (; cnt < expression.size(); cnt++)
            if (!isdigit(expression[cnt]))
                break;
        if (cnt == expression.size())
            return vector<int>(1, stoi(expression));
        
        vector<int> res;
        for (int i = 0; i < expression.size(); i++) {
            if (expression[i] == '+' || expression[i] == '-' || expression[i] == '*') {
                vector<int> leftRes = diffWaysToCompute(expression.substr(0, i)),
                            rightRes = diffWaysToCompute(expression.substr(i + 1));
                for (int leftNum : leftRes)
                    for (int rightNum : rightRes) {
                        if (expression[i] == '+')
                            res.push_back(leftNum + rightNum);
                        else if (expression[i] == '-')
                            res.push_back(leftNum - rightNum);
                        else    
                            res.push_back(leftNum * rightNum);
                    }
            }
        }
        return res;
    }
};
