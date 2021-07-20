/**
 * Definition for a binary tree node.
 * struct TreeNode {
 *     int val;
 *     TreeNode *left;
 *     TreeNode *right;
 *     TreeNode(int x) : val(x), left(NULL), right(NULL) {}
 * };
 */
class Codec {
public:

    // Encodes a tree to a single string.
    string serialize(TreeNode* root) {
        if (!root)
            return "";
        string res;
        queue<TreeNode*> level;
        level.push(root);

        while (!level.empty()) {
            TreeNode* curNode = level.front();
            level.pop();
            if (curNode == nullptr)
                res += "N";
            else {
                res += to_string(curNode->val);
                level.push(curNode->left);
                level.push(curNode->right);
            }
            res += " ";
        }
        return res;
    }

    // Decodes your encoded data to tree.
    TreeNode* deserialize(string data) {
        if (data == "")
            return nullptr;
        stringstream ss(data);
        string tempStr;
        ss >> tempStr;
        TreeNode* root = new TreeNode(stoi(tempStr));
        queue<TreeNode*> nodeQueue;
        nodeQueue.push(root);

        while (nodeQueue.size()) {
            TreeNode* curNode = nodeQueue.front();
            nodeQueue.pop();
            ss >> tempStr;
            if (tempStr[0] == 'N')
                curNode->left = nullptr;
            else {
                curNode->left = new TreeNode(stoi(tempStr));
                nodeQueue.push(curNode->left);
            }
            ss >> tempStr;
            if (tempStr[0] == 'N')
                curNode->right = nullptr;
            else {
                curNode->right = new TreeNode(stoi(tempStr));
                nodeQueue.push(curNode->right);
            }
        }
        return root;
    }
};

// Your Codec object will be instantiated and called as such:
// Codec ser, deser;
// TreeNode* ans = deser.deserialize(ser.serialize(root));
