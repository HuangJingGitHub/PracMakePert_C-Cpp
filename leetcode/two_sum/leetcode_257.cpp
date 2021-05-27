class Solution {
public:
    vector<string> binaryTreePaths(TreeNode* root) {
        vector<string> res;
        string valStr = to_string(root->val),
               prefix = valStr + "->";

        if (!root->left && !root->right)
            return vector<string>(1, valStr);
        if (root->left) {
            res = binaryTreePaths(root->left);
            for (string& str : res) 
                str = prefix + str;
        }
        if (root->right) {
            vector<string> rightRes = binaryTreePaths(root->right);
            for (string str : rightRes)
                res.push_back(prefix + str);
        }

        return res;
    }
};
