class Solution {
public:
    string convert(string s, int numRows) {
        if (numRows == 1)
            return s;

        string res;
        vector<vector<char>> table(numRows);
        
        int curRow = 0;
        bool goDown = false;
        for (int i = 0; i < s.size(); i++) {
            table[curRow].push_back(s[i]);

            if (curRow == numRows - 1 || curRow == 0)
                goDown = !goDown;
            if (goDown)
                curRow++;
            else
                curRow--;
        }

        for (int row = 0; row < numRows; row++)
            for (int col = 0; col < table[row].size(); col++)
                res += table[row][col];
        return res;
    }
};
