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
    ListNode* sortList(ListNode* head) {
        if (!head || !head->next)  // base case
            return head;
        ListNode *slowPt = head, *fastPt = head->next, *rightHead;
        while (fastPt && fastPt->next) {
            slowPt = slowPt->next;
            fastPt = fastPt->next->next;
        }
        rightHead = slowPt->next;
        slowPt->next = nullptr;
        return mergeList(sortList(head), sortList(rightHead));
    }

    ListNode* mergeList(ListNode* head1, ListNode* head2) {
        if (!head1)
            return head2;
        if (!head2)
            return head1;
        if (head1->val < head2->val) {
            head1->next = mergeList(head1->next, head2);
            return head1;
        }
        else {
            head2->next = mergeList(head1, head2->next);
            return head2;
        }
    }
};

// insertion sort O(n^2) complexity, the time limit exceeded
class Solution {
public:
    ListNode* sortList(ListNode* head) {
        if (head == nullptr || head->next == nullptr)
            return head;
            
        ListNode dummyNode = ListNode(0);
        ListNode* dummy = &dummyNode;
        dummy->next = head;
        head = head->next;
        dummy->next->next = nullptr;

        while (head) {
            ListNode *sortedHead = dummy->next, *sortedPre = dummy, *unSortedHead = head->next;
            while (sortedHead != nullptr && sortedHead->val < head->val) {
                sortedHead = sortedHead->next;
                sortedPre = sortedPre->next;
            }
            sortedPre->next = head;
            head->next = sortedHead;
            head = unSortedHead;
        }
        return dummy->next;
    }
};
