class Solution {
public:
    string alphabetBoardPath(string target) {
        string res;

        char startChar = 'a';
        for (int i = 0; i < target.size(); i++){
            char targetChar = target[i];
            addPath(res, startChar, targetChar);
            startChar = targetChar;
        }
        return res;
    }

    void addPath(string& res, char startChar, char targetChar){
        // deal with special case with 'z'
        if (startChar == 'z' && targetChar != 'z'){
            res += "U";
            addPath(res, 'u', targetChar);
            return;
        }
        else if (startChar != 'z' && targetChar == 'z'){
            addPath(res, startChar, 'u');
            res.pop_back();  // remember to delete '!' because of invocation of function.
            res += "D!";
            return;
        }

        int startInt = startChar - 'a', targetInt = targetChar - 'a';
        int moveRow = targetInt / 5 - startInt / 5, moveCol = targetInt % 5 - startInt % 5;
 
        if (moveRow > 0)
            for (int i = 0; i < moveRow; i++)
                res += "D";
        else if (moveRow < 0)
            for (int i = 0; i < -moveRow; i++)
                res += "U";
        
        if (moveCol > 0)
            for (int i = 0; i < moveCol; i++)
                res += "R";
        else if (moveCol < 0)
            for (int i = 0; i < -moveCol; i++)
                res += "L";
        res += "!";        
    }
};
