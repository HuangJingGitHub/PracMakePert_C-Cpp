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
