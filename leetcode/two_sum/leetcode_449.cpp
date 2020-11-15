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
        queue<TreeNode*> nodeQueue;
        nodeQueue.push(root);

        while (!nodeQueue.empty()){
            TreeNode *temp = nodeQueue.front();
            res.append(to_string(temp->val));
            res.append(" ");
            nodeQueue.pop();
            if (temp->left)
                nodeQueue.push(temp->left);
            if (temp->right)
                nodeQueue.push(temp->right);
        }
        return res;
    }

    // Decodes your encoded data to tree.
    TreeNode* deserialize(string data) {
        if (data.size() == 0)
            return NULL;
        stringstream io;
        io << data;
        int nodeVal;
        io >> nodeVal;
        TreeNode *head = new TreeNode(nodeVal);
        int spaceNum = count(data.begin(), data.end(), ' ');

        for (int i = 0; i < spaceNum - 1; i++){
            io >> nodeVal;
            TreeNode *node = head;
            while (true){
                if (nodeVal >= node->val && node->right)
                    node = node->right;
                else if (nodeVal >= node->val && !node->right){
                    node->right = new TreeNode(nodeVal);
                    break;
                }
                else if (nodeVal < node->val && node->left)
                    node = node->left;
                else if (nodeVal < node->val && !node->left){
                    node->left = new TreeNode(nodeVal);
                    break;
                }
            }
        }
        return head;
    }
};

// Your Codec object will be instantiated and called as such:
// Codec* ser = new Codec();
// Codec* deser = new Codec();
// string tree = ser->serialize(root);
// TreeNode* ans = deser->deserialize(tree);
// return ans;
