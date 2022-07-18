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
    ListNode* removeNthFromEnd(ListNode* head, int n) {
        ListNode dummy = ListNode(0);
        dummy.next = head;
        
        int length = 0;
        ListNode* first = head;
        while (first != NULL) {
            length++;
            first = (*first).next;
        }
        length -= n;
        first = &dummy;
        while (length > 0) {
            length--;
            first = (*first).next;
        }
        (*first).next = (*(*first).next).next;
        return dummy.next;
    }
};
