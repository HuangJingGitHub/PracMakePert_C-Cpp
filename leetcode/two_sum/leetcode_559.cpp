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
    int maxDepth(Node* root) {
        int depth = 0;
        if (!root)
            return 0;
        for (Node* it : root->children) {
            if (it)
                depth = max(depth, maxDepth(it));
        }
        depth++;
        return depth;
    }
};
