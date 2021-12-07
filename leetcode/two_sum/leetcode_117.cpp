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


// Do not use extra space, traverse in each level
class Solution {
public:
    Node* connect(Node* root) {
        if (root == nullptr)
            return root;
        
        Node* levelHead = root;
        while (levelHead != nullptr) {
            Node *curLevelNode = levelHead, *curLevelHead = curLevelNode;
            while (curLevelNode != nullptr) {
                // skip node without child
                if (curLevelNode->left == nullptr && curLevelNode->right == nullptr) {
                    curLevelNode = curLevelNode->next;
                    continue;
                }

                // find next node in the same level with child
                Node *nextNode = curLevelNode->next;
                while (nextNode != nullptr) {
                    if (nextNode->left != nullptr || nextNode->right != nullptr)
                        break;
                    nextNode = nextNode->next;
                }

                // connect
                if (curLevelNode->left != nullptr && curLevelNode->right != nullptr)
                    curLevelNode->left->next = curLevelNode->right;
                if (curLevelNode->right != nullptr && nextNode != nullptr)
                    curLevelNode->right->next = (nextNode->left != nullptr) ? nextNode->left : nextNode->right;
                else if (curLevelNode->left != nullptr && nextNode != nullptr)
                    curLevelNode->left->next = (nextNode->left != nullptr) ? nextNode->left : nextNode->right;
                
                curLevelNode = nextNode;
            }
            
            // find head node of the next level
            levelHead = nullptr;
            while (curLevelHead != nullptr) {
                if (curLevelHead->left != nullptr) {
                    levelHead = curLevelHead->left;
                    break;
                }
                else if (curLevelHead->right != nullptr) {
                    levelHead = curLevelHead->right;
                    break;
                }
                curLevelHead = curLevelHead->next;
            }
        }

        return root;
    }
};
