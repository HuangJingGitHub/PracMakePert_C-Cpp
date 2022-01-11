/*
// Definition for a Node.
class Node {
public:
    int val;
    Node* next;
    Node* random;
    
    Node(int _val) {
        val = _val;
        next = NULL;
        random = NULL;
    }
};
*/

// Use pointer as key-value in hashing.
class Solution {
public:
    Node* copyRandomList(Node* head) {
        if (!head)
            return NULL;
        Node *resHead = new Node(head->val), *resPt = resHead;
        unordered_map<Node*, Node*>  nodeMap;
         nodeMap[head] = resHead;

        while (head) {
            if (nodeMap.find(head->next) != nodeMap.end())
                resHead->next =  nodeMap[head->next];
            else if (head->next){
                resHead->next = new Node(head->next->val);
                nodeMap[head->next] = resHead->next;
            }
            if (nodeMap.find(head->random) != nodeMap.end())
                resHead->random = nodeMap[head->random];
            else if (head->random) {
                resHead->random = new Node(head->random->val);
                nodeMap[head->random] = resHead->random;
            }
            head = head->next;
            resHead = resHead->next;
        }
        return resPt;
    }
};
