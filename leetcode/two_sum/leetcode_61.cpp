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
        for (int i = 0; i < k; i++)
            rotateOnce(head);
        return head;
    }

    void rotateOnce(ListNode* head)
    {
        ListNode *originalHead = head, *phead;
        for (; head->next != NULL; head = head->next){
            if (head->next != NULL)
                phead = head;
        }

        phead->next = NULL;
        head->next = originalHead;
    }
};
