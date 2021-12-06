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
            return root;

        queue<Node*> nodeQueue;
        nodeQueue.push(root);
        Node* temp;

        while (!nodeQueue.empty()){
            int levelSize = nodeQueue.size();
            for (int i = 0; i < levelSize; i++){
                temp = nodeQueue.front();
                nodeQueue.pop();
                if (i == levelSize - 1)
                  temp->next = NULL;
                else
                  temp->next = nodeQueue.front();
                  
                if (temp->left != NULL){
                    nodeQueue.push(temp->left);
                    nodeQueue.push(temp->right);
                }
            }
        }
        return root;
    }
};


// use constant extra space, just traverse at each level
class Solution {
public:
    Node* connect(Node* root) {
        if (root == NULL)
            return root;

        Node *preLevel = root, *temp;
        while (preLevel->left != NULL){
            temp = pre;
            while(temp != NULL){
                temp->left->next = temp->right;
                if (temp->next != NULL){
                    temp->right->next = temp->next->left;
                }
                temp = temp->next;
            }
            preLevel = preLevel->left;
        }
        return root;
    }
};


// recursion
class Solution {
public:
    Node* connect(Node* root) {
        dfs(root);
        return root;
    }

    void dfs(Node* root) {
        if (root == nullptr)
            return;
        
        Node *left = root->left, *right = root->right;
        while (left != nullptr) {
            left->next = right;
            left = left->right;
            right = right->left;
        }

        dfs(root->left);
        dfs(root->right);
    }
};
