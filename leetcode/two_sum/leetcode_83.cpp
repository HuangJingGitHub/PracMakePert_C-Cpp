// Classical problem. Just pay attention.
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
    ListNode* deleteDuplicates(ListNode* head) {
        ListNode *pt = new ListNode(0), *left, *right;
        pt->next = head;
        head = pt;
        
        for ( ; pt->next; ){
            left = pt->next;
            right = left->next;
            while (right && right->val == left->val)
                right = right->next;
            left->next = right;
            pt = pt->next;
        }
        return head->next;
    }
};