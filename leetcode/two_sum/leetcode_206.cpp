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
    ListNode* reverseList(ListNode* head) {
        ListNode *pre = NULL, *next;
        while (head) {
            next = head->next;
            head->next = pre;
            pre = head;
            head = next;
        }
        return pre;
    }
};


class Solution {
public:
    ListNode* reverseList(ListNode* head) {
        if (head == nullptr)
            return head;
        ListNode dummyNode(0);
        ListNode *dummyPt = &dummyNode;
        dummyPt->next = head;

        ListNode *movePt = head->next;
        dummyPt->next->next = NULL;
        while (movePt != nullptr) {
            ListNode *tempNext = dummyPt->next, *tempMove = movePt->next;
            dummyPt->next = movePt;
            movePt->next = tempNext;
            movePt = tempMove;
        }
        return dummyPt->next;
    }
};
