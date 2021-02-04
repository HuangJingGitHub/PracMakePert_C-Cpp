/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode() : val(0), next(nullptr) {}
 *     ListNode(int x) : val(x), next(nullptr) {}
 *     ListNode(int x, ListNode *next) : val(x), next(next) {}
 * };
 */
class Solution {
public:
    ListNode* removeElements(ListNode* head, int val) {
         while (head && head->val == val)
            head = head->next;
        if (!head)
            return head;

        ListNode *pre = head, *cur = head->next;
        while (cur) {
            if (cur->val == val) {
                while (cur && cur->val == val)
                    cur = cur->next;
                pre->next = cur;
            }
            pre = pre->next;
            if (cur)
                cur = cur->next;
        }
        return head;
    }
};
