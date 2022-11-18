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
    ListNode* insertionSortList(ListNode* head) {
        if (!head)
            return NULL;

        ListNode dummyNode = ListNode(), *dummy = &dummyNode;  // To avoid potential memory leak, do not new a node.
        dummy->next = head;
        head = head->next;
        dummy->next->next = NULL;

        while (head) {
            ListNode *sortedPt = dummy->next, *sortedPtPre = dummy;
            while (sortedPt && head->val > sortedPt->val) {
                sortedPt = sortedPt->next;
                sortedPtPre = sortedPtPre->next;
            }
            sortedPtPre->next = head;
            head = head->next;
            sortedPtPre->next->next = sortedPt;
        }
        return dummy->next;
    }
};
