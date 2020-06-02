// Use stringstream and getline() to split the path by "/".
class Solution {
public:
    string simplifyPath(string path) {
        stringstream ss(path);
        vector<string> strs;
        string res, tmp;

        while(getline(ss, tmp, '/')){
            if (tmp == "" || tmp == ".")
                continue;
            else if (tmp == ".." && !strs.empty())
                strs.pop_back();
            else if (tmp != "..")
                strs.push_back(tmp);
        }

        for (string str:strs)
            res += "/" + str;
        if (res.empty())
            return "/";
        return res;
    }
};
