/*
// Definition for a Node.
class Node {
public:
    int val;
    Node* left;
    Node* right;
    Node* next;

    Node() : val(0), left(NULL), right(NULL), next(NULL) {}

    Node(int _val) : val(_val), left(NULL), right(NULL), next(NULL) {}

    Node(int _val, Node* _left, Node* _right, Node* _next)
        : val(_val), left(_left), right(_right), next(_next) {}
};
*/

class Solution {
public:
    Node* connect(Node* root) {
        if (root == NULL)
            return NULL;

        queue<Node*> nodeQueue;
        nodeQueue.push(root);
        Node* temp;
        int levelSize;

        while (!nodeQueue.empty()){
            levelSize = nodeQueue.size();
            for (int i = 0; i < levelSize; i++){
                temp = nodeQueue.front();
                nodeQueue.pop();           
                if (i != levelSize-1)
                    temp->next = nodeQueue.front();
                if (temp->left)
                    nodeQueue.push(temp->left);
                if (temp->right)
                    nodeQueue.push(temp->right);
            }
        }

        return root;
    }
};
