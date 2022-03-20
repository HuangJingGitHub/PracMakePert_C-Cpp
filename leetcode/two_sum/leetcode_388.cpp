class Solution {
public:
    int lengthLongestPath(string input) {
        vector<string> stk;
        string curStr = "";
        bool isFile = false;
        int res = 0;

        for (int i = 0; input[i]; i++) {
            int cnt = 0;
            
            if (input[i] == '\n') 
                i++;    //判断是否在dir下
            while (input[i] == '\t') {
                cnt++;
                i++;  
            }   //统计\t个数表示层数
            
            while (stk.size() > cnt) 
                stk.pop_back(); //将大于当前层数的目录全部出栈

            while (input[i] && input[i] != '\n') {
                curStr += input[i];
                if (input[i] == '.') 
                    isFile = true;
                i++ ;
            }//取出目录名或者文件名

            if (isFile) {
                int tempRes = curStr.size();
                for (int j = 0; j < stk.size(); j++) {
                    tempRes += stk[j].size() + 1;
                }
                curStr.clear();
                isFile = false;
                res = max(res, tempRes);
            }
            else {
                stk.push_back(curStr);
                curStr.clear();
            }//目录加入栈顶
            i--;
        }

        return res;
    }
};
