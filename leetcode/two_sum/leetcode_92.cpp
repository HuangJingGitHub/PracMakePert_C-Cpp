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
    ListNode* reverseBetween(ListNode* head, int m, int n) {
        ListNode *p = new ListNode(0), *pre;
        p->next = head;
        head = p;

        for (int i = 0; i < n; i++) {
            if (i < m) {
                pre = p;
                p = p->next;
            }
            else {  // Keep inserting the nodes to the back of m-1 th node
                ListNode  *tmp = p->next->next;
                p->next->next = pre->next;
                pre->next = p->next;
                p->next = tmp;
            }
        }
        
       return head->next; 
    }
};


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
    ListNode* reverseBetween(ListNode* head, int left, int right) {
        ListNode* dummy = new ListNode(0, head), *pre = dummy;
        for (int i = 1; i < left; i++)
            pre = pre->next;
        
        ListNode *pt = pre->next->next, *reverseEnd = pre->next, *nextPt = nullptr;
        for (int i = left + 1; i <= right; i++) {
            nextPt = pt->next;
            pt->next = pre->next;
            pre->next = pt;
            pt = nextPt;
        }
        reverseEnd->next = pt;

        head = dummy->next;
        delete dummy;
        return head;
    }
};
