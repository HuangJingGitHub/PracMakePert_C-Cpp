/*
// Definition for a Node.
class Node {
public:
    int val;
    vector<Node*> children;

    Node() {}

    Node(int _val) {
        val = _val;
    }

    Node(int _val, vector<Node*> _children) {
        val = _val;
        children = _children;
    }
};
*/

class Solution {
public:
    vector<vector<int>> levelOrder(Node* root) {
        vector<vector<int>> res;
        if (root == nullptr)
            return res;

        queue<Node*> levelNode;
        levelNode.push(root);
        while (levelNode.empty() == false) {
            int levelNodeNum = levelNode.size();
            vector<int> levelVec;
            for (int i = 0; i < levelNodeNum; i++) {
                Node* curNode = levelNode.front();
                levelNode.pop();
                levelVec.push_back(curNode->val);
                for (auto child : curNode->children)
                    if (child != nullptr)
                        levelNode.push(child);
            }
            res.push_back(levelVec);
        }
        return res;
    }
};
