class Solution {
public:
    bool isValidSerialization(string preorder) {
        stack<string> strStack;

        for (int i = 0, j = 0; j < preorder.size(); j++) {
            while (j < preorder.size() && preorder[j] != ',')
                j++;
            strStack.push(preorder.substr(i, j - i));
            i = j + 1;

            while (strStack.size() >= 3 && strStack.top() == "#") {
                vector<string> temp;
                for (int i = 0; i < 3; i++) {
                    temp.push_back(strStack.top());
                    strStack.pop();
                }
                if (temp[0] == "#" && temp[1] == "#" && temp[2] != "#")
                    strStack.push("#");
                else {
                    for (int i = 2; i >= 0; i--)
                        strStack.push(temp[i]);
                    break;
                }
            }
        }

        return strStack.size() == 1 && strStack.top() == "#";
    }
};


// better to use vector as stack in this case
class Solution {
public:
    bool isValidSerialization(string preorder) {
        vector<string> strStack;

        for (int i = 0, j = 0; j < preorder.size(); j++) {
            while (j < preorder.size() && preorder[j] != ',')
                j++;
            strStack.push_back(preorder.substr(i, j - i));
            i = j + 1;

            while (strStack.size() >= 3 && strStack.back() == "#") {
                int curLen = strStack.size();
                if (strStack[curLen - 1] == "#" && strStack[curLen - 2] == "#" 
                    && strStack[curLen - 3] != "#") {
                        strStack.pop_back();
                        strStack.pop_back();
                        strStack.back() = "#";
                }
                else 
                    break;
            }
        }

        return strStack.size() == 1 && strStack.front() == "#";
    }
};
