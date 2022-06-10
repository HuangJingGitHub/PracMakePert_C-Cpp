// Quite effective algorithm by using an ascending stack compared with direct O(n^2) traversing algorithm.
// See the discussion on LeetCode.
class Solution {
public:
    int largestRectangleArea(vector<int>& heights) {
        int len = heights.size(), res = 0;
        if (len == 1)
            return heights[0];
        
        stack<int> stk;
        for (int i = 0; i < len; i++){
            while (!stk.empty() && heights[stk.top()] > heights[i]){
                int h = heights[stk.top()], w = i;
                stk.pop();
                if (!stk.empty())
                    w = i - stk.top() - 1;
                res = max(res, h*w);
            }
            stk.push(i);
        }

        while(!stk.empty()) {
            int h = heights[stk.top()], w = len;
            stk.pop();
            if (!stk.empty())
                w = len - stk.top() - 1;
            res = max(res, h*w);
        }
        return res;
    }
};
