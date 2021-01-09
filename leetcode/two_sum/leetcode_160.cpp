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
    ListNode *getIntersectionNode(ListNode *headA, ListNode *headB) {        
        int lenA = 0, lenB = 0;
        ListNode *head;
        head = headA;
        while (head) {
            lenA++;
            head = head->next;
        }
        head = headB;
        while (head) {
            lenB++;
            head = head->next;
        }

        if (lenA >= lenB) 
            for (int i = 0; i < lenA - lenB; i++) 
                headA = headA->next;
        else
            for (int i = 0; i < lenB - lenA; i++)
                headB = headB->next;
        while (headA) {
            if (headA == headB)
                return headA;
            headA = headA->next;
            headB = headB->next;
        }
        return NULL;
    }
};
