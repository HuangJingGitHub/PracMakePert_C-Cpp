class Solution {
public:
    bool isPalindrome(ListNode* head) {
        ListNode *fast = head, *slow = head;
        while (fast && fast->next) {
            fast = fast->next->next;
            slow = slow->next;
        }

        ListNode* midHead = reverseList(slow);
        while (head && midHead) {
            if (head->val != midHead->val)
                return false;
            head = head->next;
            midHead = midHead->next;
        }
        return　true;
    }

    // Interesting recursion to reverese a list.
    ListNode* reverseList(ListNode* head) {
        if (!head || !head->next)
            return head;
        ListNode *newHead = reverseList(head->next);
        head->next->next = head;
        head->next = NULL;
        return newHead;
    }
};
