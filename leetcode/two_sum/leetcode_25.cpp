/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode(int x) : val(x), next(NULL) {}
 * };
 */
class Solution {
public:
    void reverseKNodes(ListNode* head, ListNode* tail){
        ListNode *preNode = head, *currentNode = head->next, *nextNode;
        while (currentNode != tail->next && currentNode != NULL){
            nextNode = currentNode->next;
            currentNode->next = preNode;
            preNode = currentNode;
            currentNode = nextNode;
        }
        head->next = nextNode;
    }

    ListNode* reverseKGroup(ListNode* head, int k) {
        if (head == NULL || head->next == NULL || k <= 1)
            return head;

        ListNode *dummy = new ListNode(0);
        dummy->next = head;
        ListNode *start = head, *end = head, *preGroupTail = dummy;
        int ListNodeNum, initialTime = 0;
        
        while (end != NULL){
            for (ListNodeNum = 0, start = end, initialTime++; ListNodeNum < k-1 && end != NULL; end = end->next, ListNodeNum++);
            if (ListNodeNum == k-1){
                reverseKNodes(start, end);
                preGroupTail->next = end;
                preGroupTail = end;
                end = end->next;
            }       
        }
        return dummy->next;    
    }
};
