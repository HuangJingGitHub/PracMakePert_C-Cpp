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
