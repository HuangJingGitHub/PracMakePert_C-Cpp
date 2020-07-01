// What is interesting is to convert this problem into the one of LeetCode 84. Cool!
class Solution {
public:
    int maximalRectangle(vector<vector<char>>& matrix) {
        if (matrix.size() == 0 || matrix[0].size() == 0)
            return 0;
        
        int row = matrix.size(), col = matrix[0].size(), res = 0;
        vector<int> heightLog(col, 0);
        for (int i = 0; i < row; i++){
            for (int j = 0; j < col; j++){
                if (matrix[i][j] == '0')
                    heightLog[j] = 0;
                else    
                    heightLog[j] += 1;
            }
            res = max(res, largestRectangleAera(heightLog));
        }
        return res;
    }

    int largestRectangleAera(vector<int>& heights){
        if (heights.empty())
            return 0;
        else if (heights.size() == 1)
            return heights[0];

        int len = heights.size(), res = 0;
        stack<int> stk;

        for (int i = 0; i < len; i++){
            while (!stk.empty() && heights[stk.top()] > heights[i]){
                int h = heights[stk.top()], width = i; 
                stk.pop();
                if (!stk.empty())
                    width = i - stk.top() - 1;
                res = max(res, h*width);
            }
            stk.push(i);
        } 

        while (!stk.empty()){
            int h = heights[stk.top()], width = len;
            stk.pop();
            if (!stk.empty())
                width = len - stk.top() - 1;
            res = max(res, h*width);
        }
        return res;
    }
};
