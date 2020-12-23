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

        ListNode* res = new ListNode();
        res->next = head;
        head = head->next;
        res->next->next = NULL;

        while (head) {
            ListNode *sortedPt = res->next, *sortedPtPre = res, *temp = head->next;
            while (sortedPt && head->val > sortedPt->val) {
                sortedPt = sortedPt->next;
                sortedPtPre = sortedPtPre->next;
            }
            sortedPtPre->next = head;
            head->next = sortedPt;
            head = temp;
        }
        return res->next;
    }
};
