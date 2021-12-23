/** classical backtracking
*/
class Solution {
public:
    vector<vector<string>> partition(string s){
        int len = s.size();
        vector<vector<string>> res;
        if (len == 0)
            return res;
        
        vector<string> path;
        backtracking(s, 0, len, path, res);
        return res;
    }
    
    void backtracking(string& s, int start, int len, vector<string>& path, vector<vector<string>>& res){
        if (start == len){
            res.push_back(path);
            return;
        }

        for (int i = start; i < len; i++){
            if (!checkPalindrome(s, start, i))
                continue;
            
            path.push_back(s.substr(start, i - start + 1));
            backtracking(s, i + 1, len, path, res);
            path.pop_back();
        }
    }

    bool checkPalindrome(string& s, int left, int right){
        while (left < right){
            if (s[left] != s[right])
                return false;
            left++;
            right--;
        }
        return true;
    }
};

class Solution {
public:
    vector<vector<string>> partition(string s) {
        vector<vector<string>> res;
        if (s.size() == 1) {
            res.push_back(vector<string>{string(1, s[0])});
            return res;
        }

        for (int i = 1; i < s.size(); i++) {
            vector<vector<string>> tempRes = partition(s.substr(0, i));
            if (isEndPalindrome(s, i)) {
                for (vector<string>& strVec : tempRes) {
                    strVec.push_back(s.substr(i));
                    res.push_back(strVec);
                }
            }
        }
        if (isEndPalindrome(s, 0))
            res.push_back(vector<string>{s});

        return res;
    }

    bool isEndPalindrome(string& s, int start) {
        int startIdx = start, endIdx = s.size() - 1;
        while (startIdx < endIdx) {
            if (s[startIdx] != s[endIdx])
                return false;
            startIdx++;
            endIdx--;
        }
        return true;
    }
};
