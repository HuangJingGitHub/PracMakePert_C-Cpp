class Solution {
public:
    string decodeString(string s) {
        string res;
        stack<char> charStk;

        for (int i = s.size() - 1; i >= 0; i--) {
            if (charStk.empty() || s[i] ！= '[') {
                charStk.push(s[i]);
            }
            if (s[i] == '[') {
                int j = i - 1;
                while (j >= 0 && s[j] >= '0' && s[j] < '9')
                    j--;
                int keyNum = stoi(s.substr(j + 1, i - j));
                string keyStr;
                while (charStk.top() != ']') {
                    
                }
            }
        }
    }
};
