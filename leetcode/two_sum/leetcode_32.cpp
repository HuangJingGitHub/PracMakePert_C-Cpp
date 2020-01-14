class Solution {
public:
    int longestValidParentheses(string s) {
        if (s.size() <= 1)
            return 0;
        
        int longestValid = 0, counter = 0;    
        vector<char> stackStr;
        vector<int> validMap(s.size(), 0), strIndex;
        stackStr.push_back(s[0]);
        strIndex.push_back(0);

        for (int i = 1; i < s.length(); i++)
        {
            switch(s[i])
            {
                case '(':
                    stackStr.push_back(s[i]);
                    strIndex.push_back(i);
                    break;

                case ')':
                    if (stackStr.size() > 0 && stackStr[stackStr.size() - 1] == '(')
                    {
                        validMap[strIndex[strIndex.size() - 1]] = 1;
                        validMap[i] = 1;
                        stackStr.pop_back();
                        strIndex.pop_back();
                    }
                    else
                    {
                        stackStr.push_back(s[i]);
                        strIndex.push_back(i);
                    }
                    break;                                           
            }
        }

        for (int i = 0; i < validMap.size(); i++)
        {
            if (validMap[i] == 0)
            {
                longestValid = longestValid > counter ? longestValid : counter;
                counter = 0;
            }
            else if ( i == validMap.size() - 1)
            {
                counter += 1;
                longestValid = longestValid > counter ? longestValid : counter;
            }
            else 
                counter += 1;
        }
        return longestValid;
    }
};
