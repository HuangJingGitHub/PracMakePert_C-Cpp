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
    ListNode* reverseBetween(ListNode* head, int m, int n) {
        ListNode *p = new ListNode(0), *pre;
        p->next = head;
        head = p;

        for (int i = 0; i < n; i++){
            if (i < m){
                pre = p;
                p = p->next;
            }
            else{
                ListNode  *tmp = p->next->next;
                p->next->next = pre->next;
                pre->next = p->next;
                p->next = tmp;
            }
        }
       return head->next; 
    }
};
