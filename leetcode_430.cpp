/*
// Definition for a Node.
class Node {
public:
    int val;
    Node* prev;
    Node* next;
    Node* child;
};
*/

class Solution {
public:
    Node* flatten(Node* head) {
        Node* pt = head;
        while (pt != nullptr) {
            if (pt->child != nullptr) 
                helper(pt, pt->next);
            pt = pt->next;
        }

        return head;
    }

    void helper(Node* prevNode, Node* nextNode) {  
        prevNode->next = prevNode->child;
        prevNode->child->prev = prevNode;
        prevNode->child = nullptr;

        Node* pt = prevNode->next;
        while (pt->next != nullptr)
            pt = pt->next;
        pt->next = nextNode;
        if (nextNode != nullptr)
            nextNode->prev = pt;

        pt = prevNode->next;
        while (pt != nextNode) {
            if (pt->child != nullptr)
                helper(pt, pt->next);
            pt = pt->next;
        }
    }
};
