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
    ListNode* oddEvenList(ListNode* head) {
        if (head == nullptr)
            return head;

        ListNode *evenHeadLog = head->next;
        ListNode *oddHead = head, *evenHead = head->next;
        while (oddHead->next && evenHead->next) {
            oddHead->next = evenHead->next;
            oddHead = oddHead->next;
            evenHead->next = oddHead->next;
            evenHead = evenHead->next;
        }
        if (oddHead)
            oddHead->next = nullptr;
        oddHead->next = evenHeadLog;
        return head;
    }
};
