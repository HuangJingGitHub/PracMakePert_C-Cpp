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
    ListNode* mergeTwoLists(ListNode* l1, ListNode* l2) {
     ListNode* head = new ListNode(0), *currentNode = head;
     while (l1 != NULL && l2 != NULL)
     {
         if ((*l1).val < (*l2).val)
         {
             (*currentNode).next = l1;
             l1 = (*l1).next;
         }
         else
         {
             (*currentNode).next = l2;
             l2 = (*l2).next;
         }
         currentNode = (*currentNode).next;
     }

     if (l1 == NULL)
     {
        (*currentNode).next = l2;
     }   
    else
    {
        (*currentNode).next = l1;
    }
    return head->next;
    }
};
