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
    ListNode* addTwoNumbers(ListNode* l1, ListNode* l2) {
        ListNode  *dummy = new ListNode(0), *resPt = dummy;

        int carry = 0;
        while (l1 && l2) {
            int curDigit = l1->val + l2->val + carry;
            carry = curDigit / 10;
            curDigit %= 10;
            resPt->next = new ListNode(curDigit);
            resPt = resPt->next;
            l1 = l1->next;
            l2 = l2->next;
        }

        ListNode* l3 = l1 == nullptr ? l2 : l1;
        while (l3) {
            int curDigit = l3->val + carry;
            carry = curDigit / 10;
            curDigit %= 10;
            resPt->next = new ListNode(curDigit);
            resPt = resPt->next;
            l3 = l3->next;
        }
        if (carry != 0)
            resPt->next = new ListNode(carry);
        
        resPt = dummy->next;
        delete dummy;
        return resPt;
    }
};
