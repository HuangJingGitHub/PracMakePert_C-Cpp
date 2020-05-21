// 1) Passing of pointer in funciton is passing by value by default.
// 2) Some details.
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
    ListNode* rotateRight(ListNode* head, int k) {
        if (head == NULL)
            return head;
        
        int len = 1;
        for (auto pt = head; pt->next != NULL; pt = pt->next, len++);

        int times = k % len;
        for (int i = 0; i < times; i++)
            rotateOnce(head);
        
        return head;
    }

    void rotateOnce(ListNode*& head)
    {
        ListNode *originalHead = head, *phead = head;
        for (; head->next != NULL; head = head->next){
                phead = head;
        }
        
        phead->next = NULL;
        if (head != phead) 
            head->next = originalHead;
    }
};
