// Direct method
class Solution {
public:
    vector<string> restoreIpAddresses(string s) {
        vector<string> res;
        int n = s.size();

        for (int i = 0; i < 3; i++)
            for (int j = i+1; j < i+4; j++)
                for (int k = j+1; k < j+4; k++){
                    if (i < n && j < n && k < n){
                        string s1 = s.substr(0, i+1),
                               s2 = s.substr(i+1, j-i),
                               s3 = s.substr(j+1, k-j),
                               s4 = s.substr(k+1);
                        // cout << s1 << " " << s2 << " " << s3 << " " << s4 << endl;
                        if (check(s1) && check(s2) && check(s3) && check(s4))
                            res.push_back(s1 + "." + s2 + "." + s3 + "." + s4);
                    }
                }

        return res;
    }

    bool check(string s)
    {
        if (s.size() == 0 || s.size() > 3 || (s[0] == '0' && s.size() > 1) || stoi(s) > 255)
            return false;
        return true;
    }
};


class Solution {
public:
    vector<string> restoreIpAddresses(string s) {
        vector<string> res, path;
        backtrack(s, 0, res, path);
        return res;
    }
    
    void backtrack(string& s, int startIdx, vector<string>& res, vector<string>& path) {
        if (startIdx == s.size() && path.size() == 4) {
            string temp = path[0];
            for (int i = 1; i < 4; i++)
                temp += ("." + path[i]);
            res.push_back(temp);
            return;
        }
        else if (startIdx < s.size() && path.size() == 4)
            return;
        else if (startIdx >= s.size())
            return;

        for (int len = 1; len <= 3; len++) {
            string tempStr = s.substr(startIdx, len);
            if ((tempStr[0] == '0' && len > 1) || (stoi(tempStr) > 255))
                continue;
            path.push_back(tempStr);
            backtrack(s, startIdx + len, res, path);
            path.pop_back();
        }
    }
};
