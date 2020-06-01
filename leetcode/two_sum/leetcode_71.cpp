class Solution {
public:
    string simplifyPath(string path) {
        vector<char> pathStack;

        pathStack.push_back(path[0]);
        for (int i = 1; i < path.size(); i++){
            if (path[i] == '.'){
                if (pathStack.back() == '.'){
                    pathStack.push_back();
                    pathStack.push_back();
                    pathStack.push_back();
                }
                else
                    continue;
            }
            if (path[i] == '/'){
                if (pathStack.back() == '/')
                    continue;
                else
                    pathStack.push_back('/');
            }
            
        }
    }
};
